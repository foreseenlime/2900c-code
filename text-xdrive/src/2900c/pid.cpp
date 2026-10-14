#include "main.h"

PID::PID(
    double kP, double kI, double kD,
    double kP_turn, double kD_turn,
    double max_integral
):
kP(kP), kI(kI), kD(kD),
kP_turn(kP_turn), kD_turn(kD_turn),
max_integral(max_integral)
{
    // movement pid vars
    integral_x = 0.0; integral_y = 0.0;
    derivative_x = 0.0; derivative_y = 0.0;
    prev_error_x = 0.0; prev_error_y = 0.0;

    // turning pid vars
    derivative_turn = 0.0; prev_error_turn = 0.0;

    // errors
    error = 0.0; turn_error = 0.0;
}

void PID::move_to_pose(Pose pose, double timeout) {
    double start_angle = Odom::global_point.heading;
    double time_finished = pros::millis() + timeout;

    while(time_finished > pros::millis()) {
        // calculate distance and angle to target
        error = sqrt(pow((pose.x - Odom::global_point.x), 2) + pow((pose.y - Odom::global_point.y), 2));
        turn_error = atan2((pose.y - Odom::global_point.y), (pose.x - Odom::global_point.x));
        // relative angle
        // does this work as intended??
        turn_error = angle_wrap(turn_error + Odom::global_point.heading);

        // get x and y components of error
        double error_x = error * cos(turn_error);
        double error_y = error * sin(turn_error);

        // // need to figure out how to do integral (maybe use abs values??) since the error might be able to be < 0 normally due to the trig functions
        // // sum error values into integral
        // // commented out cause idk if this can work and i need to finish turning pid
        // integral_x += error_x;
        // integral_y += error_y;
        // // reset integral if error passess setpoint
        // if(error_x <= 0) {integral_x = 0;}
        // if(error_y <= 0) {integral_x = 0;}
        // // limit integral if it goes too high
        // if(integral_x > max_integral) {integral_x = max_integral;}
        // if(integral_y > max_integral) {integral_y = max_integral;}

        // derivative
        derivative_x = error_x - prev_error_x;
        derivative_y = error_y - prev_error_y;
        // set previous error
        prev_error_x = error_x;
        prev_error_y = error_y;

        // get movement vars
        // hopefully this scales correctly to motor values
        double strafe = error_x * kP + integral_x * kI + derivative_x * kD;
        double forward = error_y * kP + integral_y * kI + derivative_y * kD;

        // turning pid
        // get angle from current heading to pose heading (0-2pi radians)
        double difference = get_heading(Odom::global_point.heading - pose.heading);
        // get the shortest angle between the current heading and pose heading (0-pi radians)
        if(difference <= M_PI) {turn_error = difference;} 
        else{turn_error = (2 * M_PI) - difference;}

        // turning derivative
        derivative_turn = turn_error - prev_error_turn;
        // set previous error
        prev_error_turn = turn_error;

        double turn = turn_error * kP_turn + derivative_turn * kD_turn;

        Motor_Inputs motor_inputs = {
            forward + strafe + turn, // left front
            forward - strafe + turn, // left back
            forward - strafe - turn, // right front
            forward + strafe - turn // right back
        };
        motor_inputs = scale_to_motor(motor_inputs);

        drivetrain.set_drive_state(motor_inputs);

        pros::delay(10);
    }
    // reset drivetrain state after movement is finished
    drivetrain.set_drive_state(Motor_Inputs{0, 0, 0, 0});
}