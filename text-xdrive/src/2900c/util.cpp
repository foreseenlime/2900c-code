#include "main.h"

int sign(double num) {
    if (num >= 0) {return 1;}
    else {return -1;}
}

// converts linear joystick inputs to nonlinear motor outputs
// explanation goes here
double linear_to_squared(double input, double max_input) {
    return (pow(input, 2) * sign(input)) / max_input;
}

// converts joystick values to motor output values
double joystick_to_motor(double joystick, double max_input) {
    return joystick * (max_input / 127);
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