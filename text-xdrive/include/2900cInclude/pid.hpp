#pragma once
#include "main.h"

class PID {
    private:
        double kP, kI, kD;
        double integral_x, integral_y, max_integral;
        double derivative_x, derivative_y;
        double prev_error_x, prev_error_y;

        double kP_turn, kD_turn;
        double derivative_turn, prev_error_turn;

        double error, turn_error;

    public:
        PID(
            double kP, double kI, double kD,
            double kP_turn, double kD_turn,
            double max_integral
        );
        void move_to_pose(Pose pose, double timeout);
};