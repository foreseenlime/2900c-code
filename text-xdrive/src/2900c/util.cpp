#include "main.h"

int sign(double num) {
    if(num >= 0) {return 1;}
    else {return -1;}
}

// converts linear joystick inputs to nonlinear motor outputs
// explanation goes here
double linear_to_squared(double input, double max_input) {
    return (pow(input, 2) * sign(input)) / max_input;
}

// converts joystick values to motor output values
Motor_Inputs scale_to_motor(Motor_Inputs motor_inputs) {
    // find largest of motor values
    double max_val = std::max({
        motor_inputs.left_front, motor_inputs.left_back, 
        motor_inputs.right_front, motor_inputs.right_back
    });

    // scale to the valid motor range
    if (max_val > 127) {
        double scale = 127 / max_val;
        motor_inputs.left_front *= scale;
        motor_inputs.left_back *= scale;
        motor_inputs.right_front *= scale;
        motor_inputs.right_back *= scale;
    }

    return motor_inputs;
}

// convert degrees to radians
double get_rad(double deg) {
    return (deg / 180) * M_PI;
}

// convert radians to degrees
double get_deg(double rad) {
    return (rad / M_PI) * 180;
}

// convert rotation sensor degrees to distance
// distance = rotations * circumference
double deg_to_dist(double deg, double diameter) {
    return (deg / 360) * (diameter * M_PI);
}

double angle_wrap(double angle) {
    while(angle < -M_PI) {angle += 2 * M_PI;}
    while(angle > M_PI) {angle -= 2 * M_PI;}
    return angle;
}

double get_heading(double angle) {
    while(angle < 0) {angle += 2 * M_PI;}
    while(angle > 2 * M_PI) {angle -= 2 * M_PI;}
    return angle;
}