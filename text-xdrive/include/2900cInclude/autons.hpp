#pragma once
#include "main.h"

namespace autons {
    /**
    * @brief Move the drivetrain in one of four directions
    * 
    * @param direction Direction of movement (F, B, L, R)
    * @param speed Speed of movement (-127 - 127)
    */
    void drive(char direction, double speed);

    /**
     * @brief Rotate the drivetrain either clockwise or anticlockwise
     * 
     * @param direction Direction of rotation (C, A)
     * @param speed Speed of rotation (-127 - 127)
     */
    void rotate(char direction, double speed);

    void stop();

    void toggle_only();
}
