#include "main.h"

namespace Odom {
    // global coordinates
    Pose global_point = {0, 0, 0};
    Pose prev_global_point = {0, 0, 0};
    Pose delta_global_point = {0, 0, 0};

    // local coordinates
    Pose delta_local_point = {0, 0, 0};

    // sensor values
    // encoders
    Rotation_vals rotation_val = {0, 0};
    Rotation_vals prev_rotation_val = {0, 0};
    Rotation_vals delta_rotation_val = {0, 0};

    // imu
    double heading = 0;
    double prev_heading = 0;
    double delta_heading = 0;

    // update sensors
    void update_sensors() {
        // get rotation sensor values in inches
        rotation_val.horizontal = deg_to_dist((drivetrain.horizontal.get_position() / 100), TRACKER_CIRCUMFERENCE);
        rotation_val.vertical = deg_to_dist((drivetrain.vertical.get_position() / 100), TRACKER_CIRCUMFERENCE);

        // get change in rotation sensor values
        delta_rotation_val.horizontal = rotation_val.horizontal - prev_rotation_val.horizontal;
        delta_rotation_val.vertical = rotation_val.vertical - prev_rotation_val.vertical;

        // set previous rotation values to current ones
        prev_rotation_val.horizontal = rotation_val.horizontal;
        prev_rotation_val.vertical = rotation_val.vertical;

        // get imu heading
        heading = get_rad(drivetrain.imu.get_heading());
        // get change in imu heading
        delta_heading = heading - prev_heading;
        // set previous heading to current one
        prev_heading = heading;
    }

    void update_position() {}
}