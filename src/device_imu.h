/*
    Copyright PNDBotics 2026
    此设备目前不暴露到外部
*/
#pragma once
#include "fleet/fleet.h"

/*
  原极 IMU 协议，详见 https://forsense.yuque.com/org-wiki-forsense-dohrz0/egmla1/lcxf6pnuwdxsr5ut
*/

namespace fleet {

struct quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct ypr {
    float yaw;
    float pitch;
    float roll;
};

struct vec3 {
    float x;
    float y;
    float z;
};

struct Imu {
    quaternion quaternion;
    vec3 gyroscope_rps;
    vec3 accelerometer_mps2;
    ypr ypr;
    int16_t temperature;
};

} // namespace fleet
