#pragma once
#include "main.h"

struct Pose {
    double x, y, heading;
};

struct Rotation_vals {
    double horizontal, vertical;
};

namespace Odom {
    // global coordinates
    extern Pose global_point; // x, y, heading
    extern Pose prev_global_point;
    extern Pose delta_global_point; // change in x, y, angle

    // local coordinates
    extern Pose delta_local_point; // change in x, y

    // sensor values
    // rotation sensors
    extern Rotation_vals rotation_val; // rotation of horizontal, vertical rotation sensors
    extern Rotation_vals prev_rotation_val; 
    extern Rotation_vals delta_rotation_val; // change in horizontal, vertical rotation sensors
    // imu
    extern double heading;
    extern double prev_heading;
    extern double delta_heading; // change in heading

    // functions
    void update_sensors();
    void update_position();
    void run_odometry();
}