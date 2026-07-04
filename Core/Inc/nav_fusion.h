#ifndef NAV_FUSION_H
#define NAV_FUSION_H

#include "flight_types.h"

typedef struct {
    float position;
    float velocity;
    float covariance[2][2];
} NavAxisKalman_t;

typedef struct {
    NavAxisKalman_t north;
    NavAxisKalman_t east;
    double origin_latitude;
    double origin_longitude;
    uint32_t last_gps_fix_tick;
    bool initialized;
} NavFusion_t;

void NavFusion_Init(NavFusion_t *filter);
void NavFusion_Predict(NavFusion_t *filter, const Vector3f_t *body_accel,
                       const Orientation_t *attitude, float dt);
void NavFusion_CorrectGPS(NavFusion_t *filter, const GPS_Data_t *gps,
                          uint32_t fix_tick);
void NavFusion_GetData(const NavFusion_t *filter, Nav_Data_t *data);

#endif
