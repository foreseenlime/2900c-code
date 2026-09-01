#pragma once
#include "main.h"

namespace autons {
    /**
    * @brief Move the drivetrain in one of four directions
    * 
    * @param direction Direction of movement (F, B, L, R)
    * @param speed Speed of movement (-127 - 127)
    */
    void drive_move(char direction, double speed);

    /**
     * @brief Rotate the drivetrain either clockwise or anticlockwise
     * 
     * @param direction Direction of rotation (C, A)
     * @param speed Speed of rotation (-127 - 127)
     */
    void drive_rotate(char direction, double speed);

    /**
     * @brief stop all drivetrain movement
     */
    void drive_stop();

    // --------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------

    void toggle_only();
    void score_preload();
}
