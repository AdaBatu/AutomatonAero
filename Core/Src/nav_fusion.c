#include "nav_fusion.h"
#include <math.h>
#include <string.h>

#define EARTH_RADIUS_M          6378137.0
#define DEG_TO_RAD_D            0.017453292519943295
#define RAD_TO_DEG_D            57.29577951308232
#define ACCEL_NOISE_VARIANCE    4.0f
#define ACCEL_BIAS_WALK_VARIANCE 0.01f
#define GPS_POSITION_VARIANCE   25.0f
#define GPS_VELOCITY_VARIANCE   1.0f
#define GPS_STATIONARY_VARIANCE 0.04f
#define GPS_STATIONARY_SPEED    0.75f
#define MAX_HORIZONTAL_ACCEL    30.0f

static void axis_predict(NavAxisKalman_t *axis, float acceleration, float dt)
{
    float dt2 = dt * dt;
    float corrected_accel = acceleration - axis->accel_bias;
    axis->position += axis->velocity * dt + 0.5f * corrected_accel * dt2;
    axis->velocity += corrected_accel * dt;

    /* State is [position, velocity, accelerometer bias].  Keeping bias in the
       filter lets GPS velocity corrections remove constant IMU/installation
       error instead of allowing it to become unbounded speed drift. */
    const float f[3][3] = {
        {1.0f, dt, -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f, 1.0f}
    };
    float fp[3][3] = {{0}};
    float predicted[3][3] = {{0}};
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t col = 0U; col < 3U; ++col) {
            for (uint8_t k = 0U; k < 3U; ++k)
                fp[row][col] += f[row][k] * axis->covariance[k][col];
        }
    }
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t col = 0U; col < 3U; ++col) {
            for (uint8_t k = 0U; k < 3U; ++k)
                predicted[row][col] += fp[row][k] * f[col][k];
        }
    }
    float accel_q[3] = {0.5f * dt2, dt, 0.0f};
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t col = 0U; col < 3U; ++col) {
            predicted[row][col] += accel_q[row] * accel_q[col] *
                                   ACCEL_NOISE_VARIANCE;
            axis->covariance[row][col] = predicted[row][col];
        }
    }
    axis->covariance[2][2] += dt * ACCEL_BIAS_WALK_VARIANCE;
}

static void axis_correct(NavAxisKalman_t *axis, float measurement,
                         uint8_t state_index, float variance)
{
    float innovation = measurement -
                       (state_index == 0U ? axis->position : axis->velocity);
    float s = axis->covariance[state_index][state_index] + variance;
    if (s <= 0.0f) return;
    float gain[3];
    float measured_row[3];
    for (uint8_t i = 0U; i < 3U; ++i) {
        gain[i] = axis->covariance[i][state_index] / s;
        measured_row[i] = axis->covariance[state_index][i];
    }
    axis->position += gain[0] * innovation;
    axis->velocity += gain[1] * innovation;
    axis->accel_bias += gain[2] * innovation;
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t col = 0U; col < 3U; ++col)
            axis->covariance[row][col] -= gain[row] * measured_row[col];
    }
    /* Roundoff can slowly make P asymmetric on a microcontroller. */
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t col = row + 1U; col < 3U; ++col) {
            float average = 0.5f * (axis->covariance[row][col] +
                                    axis->covariance[col][row]);
            axis->covariance[row][col] = average;
            axis->covariance[col][row] = average;
        }
    }
}

void NavFusion_Init(NavFusion_t *filter)
{
    memset(filter, 0, sizeof(*filter));
    filter->north.covariance[0][0] = 100.0f;
    filter->north.covariance[1][1] = 25.0f;
    filter->north.covariance[2][2] = 4.0f;
    filter->east = filter->north;
}

void NavFusion_Predict(NavFusion_t *filter, const Vector3f_t *a,
                       const Orientation_t *o, float dt)
{
    if (!filter->initialized || dt <= 0.0f || dt > 0.1f) return;
    float cr = cosf(o->roll), sr = sinf(o->roll);
    float cp = cosf(o->pitch), sp = sinf(o->pitch);
    float cy = cosf(o->yaw), sy = sinf(o->yaw);
    float north_accel = cy * cp * a->x + (cy * sp * sr - sy * cr) * a->y +
                        (cy * sp * cr + sy * sr) * a->z;
    float east_accel = sy * cp * a->x + (sy * sp * sr + cy * cr) * a->y +
                       (sy * sp * cr - cy * sr) * a->z;
    if (!isfinite(north_accel) || !isfinite(east_accel)) return;
    if (north_accel > MAX_HORIZONTAL_ACCEL) north_accel = MAX_HORIZONTAL_ACCEL;
    if (north_accel < -MAX_HORIZONTAL_ACCEL) north_accel = -MAX_HORIZONTAL_ACCEL;
    if (east_accel > MAX_HORIZONTAL_ACCEL) east_accel = MAX_HORIZONTAL_ACCEL;
    if (east_accel < -MAX_HORIZONTAL_ACCEL) east_accel = -MAX_HORIZONTAL_ACCEL;
    axis_predict(&filter->north, north_accel, dt);
    axis_predict(&filter->east, east_accel, dt);
}

void NavFusion_CorrectGPS(NavFusion_t *filter, const GPS_Data_t *gps,
                          uint32_t fix_tick)
{
    if (!gps->fix_valid || fix_tick == 0U || fix_tick == filter->last_gps_fix_tick)
        return;
    double lat_rad = (double)gps->latitude * DEG_TO_RAD_D;
    if (!filter->initialized) {
        filter->origin_latitude = gps->latitude;
        filter->origin_longitude = gps->longitude;
        filter->initialized = true;
    }
    double north = ((double)gps->latitude - filter->origin_latitude) *
                   DEG_TO_RAD_D * EARTH_RADIUS_M;
    double east = ((double)gps->longitude - filter->origin_longitude) *
                  DEG_TO_RAD_D * EARTH_RADIUS_M * cos(lat_rad);
    bool stationary = gps->speed < GPS_STATIONARY_SPEED;
    float course = gps->course * (float)DEG_TO_RAD_D;
    float vn = stationary ? 0.0f : gps->speed * cosf(course);
    float ve = stationary ? 0.0f : gps->speed * sinf(course);
    float velocity_variance = stationary ? GPS_STATIONARY_VARIANCE :
                                           GPS_VELOCITY_VARIANCE;
    axis_correct(&filter->north, (float)north, 0U, GPS_POSITION_VARIANCE);
    axis_correct(&filter->east, (float)east, 0U, GPS_POSITION_VARIANCE);
    axis_correct(&filter->north, vn, 1U, velocity_variance);
    axis_correct(&filter->east, ve, 1U, velocity_variance);
    filter->last_gps_fix_tick = fix_tick;
}

void NavFusion_GetData(const NavFusion_t *filter, Nav_Data_t *data)
{
    memset(data, 0, sizeof(*data));
    if (!filter->initialized) return;
    data->latitude = filter->origin_latitude +
                     (double)filter->north.position / EARTH_RADIUS_M * RAD_TO_DEG_D;
    double latitude_rad = data->latitude * DEG_TO_RAD_D;
    double longitude_scale = EARTH_RADIUS_M * cos(latitude_rad);
    data->longitude = filter->origin_longitude +
                      (double)filter->east.position / longitude_scale * RAD_TO_DEG_D;
    data->velocity_north = filter->north.velocity;
    data->velocity_east = filter->east.velocity;
    data->speed = sqrtf(data->velocity_north * data->velocity_north +
                        data->velocity_east * data->velocity_east);
    data->valid = true;
}
