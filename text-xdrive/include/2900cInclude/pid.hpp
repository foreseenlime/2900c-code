#pragma once
#include "main.h"

class PID {
    private:
        double kP, kI, kD;

    public:
        PID(double kP, double kI, double kD);
        void move(double x, double y, double heading);
};