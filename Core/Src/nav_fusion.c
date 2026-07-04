#include "nav_fusion.h"
#include <math.h>
#include <string.h>

#define EARTH_RADIUS_M          6378137.0
#define DEG_TO_RAD_D            0.017453292519943295
#define RAD_TO_DEG_D            57.29577951308232
#define ACCEL_NOISE_VARIANCE    4.0f
#define GPS_POSITION_VARIANCE   25.0f
#define GPS_VELOCITY_VARIANCE   1.0f

static void axis_predict(NavAxisKalman_t *axis, float acceleration, float dt)
{
    float dt2 = dt * dt;
    axis->position += axis->velocity * dt + 0.5f * acceleration * dt2;
    axis->velocity += acceleration * dt;

    float p00 = axis->covariance[0][0];
    float p01 = axis->covariance[0][1];
    float p10 = axis->covariance[1][0];
    float p11 = axis->covariance[1][1];
    axis->covariance[0][0] = p00 + dt * (p10 + p01) + dt2 * p11 +
                              0.25f * dt2 * dt2 * ACCEL_NOISE_VARIANCE;
    axis->covariance[0][1] = p01 + dt * p11 +
                              0.5f * dt2 * dt * ACCEL_NOISE_VARIANCE;
    axis->covariance[1][0] = axis->covariance[0][1];
    axis->covariance[1][1] = p11 + dt2 * ACCEL_NOISE_VARIANCE;
}

static void axis_correct(NavAxisKalman_t *axis, float measurement,
                         uint8_t state_index, float variance)
{
    float innovation = measurement -
                       (state_index == 0U ? axis->position : axis->velocity);
    float s = axis->covariance[state_index][state_index] + variance;
    if (s <= 0.0f) return;
    float k0 = axis->covariance[0][state_index] / s;
    float k1 = axis->covariance[1][state_index] / s;
    float row0 = axis->covariance[state_index][0];
    float row1 = axis->covariance[state_index][1];
    axis->position += k0 * innovation;
    axis->velocity += k1 * innovation;
    axis->covariance[0][0] -= k0 * row0;
    axis->covariance[0][1] -= k0 * row1;
    axis->covariance[1][0] -= k1 * row0;
    axis->covariance[1][1] -= k1 * row1;
}

void NavFusion_Init(NavFusion_t *filter)
{
    memset(filter, 0, sizeof(*filter));
    filter->north.covariance[0][0] = 100.0f;
    filter->north.covariance[1][1] = 25.0f;
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
    float course = gps->course * (float)DEG_TO_RAD_D;
    float vn = gps->speed * cosf(course);
    float ve = gps->speed * sinf(course);
    axis_correct(&filter->north, (float)north, 0U, GPS_POSITION_VARIANCE);
    axis_correct(&filter->east, (float)east, 0U, GPS_POSITION_VARIANCE);
    axis_correct(&filter->north, vn, 1U, GPS_VELOCITY_VARIANCE);
    axis_correct(&filter->east, ve, 1U, GPS_VELOCITY_VARIANCE);
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
