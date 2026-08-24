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

/**
 * @brief Converts joystick input values to motor output values
 */
// Good for if the drivetrain is using something like rpm or 
// voltage and needs converting from the joystick values
double joystick_to_motor(double joystick, double max_input);