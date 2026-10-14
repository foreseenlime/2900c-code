#include "main.h"

namespace Odom {
    // global coordinates
    Pose global_point = {0, 0, 0};
    Pose delta_global_point = {0, 0, 0};

    // local coordinates
    Pose local_point = {0, 0, 0};
    Pose delta_local_point = {0, 0, 0};

    // sensor values
    // encoders
    Rotation_vals rotation_val = {0, 0};
    Rotation_vals prev_rotation_val = {0, 0};
    Rotation_vals delta_rotation_val = {0, 0};

    // imu
    double heading = 0;
    double prev_heading = 0;
    double mid_heading = 0;
    double delta_heading = 0;

    // update sensors
    void update_sensors() {
        // get rotation sensor values in inches
        rotation_val.horizontal = deg_to_dist((drivetrain.horizontal.get_position() / 100), TRACKER_CIRCUM);
        rotation_val.vertical = deg_to_dist((drivetrain.vertical.get_position() / 100), TRACKER_CIRCUM);

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

        // prevent large delta heading ranges on wraparound
        if (delta_heading > M_PI) {
            delta_heading -= 2 * M_PI;
        }
        else if (delta_heading < -M_PI) {
            delta_heading += 2 * M_PI;
        }

        // need heading DURING movement as opposed to AFTER for proper calculations
        mid_heading = prev_heading + (delta_heading / 2);
        // set previous heading to current one
        prev_heading = heading;
    }

    void update_position() {
        // robot movement = recorded movement +/- encoder movement caused by rotation
        delta_local_point.x = delta_rotation_val.horizontal + (HORIZONTAL_OFFSET * delta_heading);
        delta_local_point.y = delta_rotation_val.vertical + (VERTICAL_OFFSET * delta_heading);

        // local points for the sake of debugging
        local_point.x += delta_local_point.x;
        local_point.y += delta_local_point.y;

        // delta X = horizontal movement measured by horizontal wheel +/- horizontal movement measured by vertical wheel
        delta_global_point.x = (delta_local_point.y * sin(mid_heading)) + (delta_local_point.x * cos(mid_heading));
        // delta Y = vertical movement measured by horizontal wheel +/- vertical movement measured by vertical wheel
        delta_global_point.y = (delta_local_point.y * cos(mid_heading)) - (delta_local_point.x * sin(mid_heading));

        // global x, y, heading
        global_point.x += delta_global_point.x;
        global_point.y += delta_global_point.y;
        global_point.heading = heading;
    }

    // odom thread
    void run_odometry() {
        while(true) {
            // reset loop if imu is still calibrating
            if(drivetrain.imu.is_calibrating()){continue;}

            console.clear();
            Odom::update_sensors();
            Odom::update_position();

            console.printf(
                "LOCAL: X=%.3lf Y=%.3lf\nGLOBAL: X=%.3lf Y=%.3lf\nHEADING=%.3lf",
                local_point.x, local_point.y,
                global_point.x, global_point.y,
                get_deg(global_point.heading)
            );

            // console.printf(
            //     "raw + correction\n%.3lf + %.3lf",
            //     delta_rotation_val.horizontal, (HORIZONTAL_OFFSET * delta_heading)
            // );

            // slower delays bugs out rotation sensors
            pros::delay(30);
        }
    }
}