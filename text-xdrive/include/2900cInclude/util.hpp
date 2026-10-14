#pragma once

/**
 * @brief Returns the sign of a number (1, -1)
 */
int sign(double num);

/**
 * @brief Converts linear input values to nonlinear (squared) motor output values
 */
// Makes small joystick inputs more small, so it is easier to make small movements
double linear_to_squared(double input, double max_input);

struct Motor_Inputs {
    double left_front, left_back, right_front, right_back;
};

/**
 * @brief Scales inputs to the accepted range that motors can accept
 */
Motor_Inputs scale_to_motor(Motor_Inputs motor_inputs);

/**
 * @brief Converts degrees to radians
 */
double get_rad(double deg);

/**
 * @brief Converts radians to degrees
 */
double get_deg(double rad);

/**
 * @brief Convert rotation sensor degrees to distance
 */
double deg_to_dist(double deg, double diameter);

/**
 * @brief Struct for a position on the field
 * 
 * @param x Horizontal position, in inches
 * @param y Vertical position, in inches
 * @param heading Direction that robot is facing, in radians
 */
struct Pose {
    double x, y, heading;
};

/**
 * @brief Wraps angle into a range of -pi to pi radians
 */
double angle_wrap(double angle);

/**
 * @brief Wraps angle into a range of 0 to 2pi radians
 */
double get_heading(double angle);