#pragma once
#include "main.h"

namespace subsystems {

    class Drivetrain {

        private:
            pros::Motor left_front;
            pros::Motor left_back;
            pros::Motor right_front;
            pros::Motor right_back;

        public:
            // constructor
            Drivetrain(
                int left_front_port,
                int left_back_port, 
                int right_front_port,
                int right_back_port
            );

            void set_drive_state(float left_front_p, float left_back_p, float right_front_p, float right_back_p);
            void drive_functions();
    };

    class Claw {

        private:
            pros::Motor claw_motor;
            int claw_velocity = 180;
            bool open = false;

        public:
            // constructor
            Claw(int claw_port);

            void set_claw_state(bool open);
            void claw_functions();

    };

    class Lift {

        private:
            pros::Motor dr4b1;
            pros::Motor dr4b2;

            pros::MotorGroup dr4b;

        public:
            // constructor
            Lift(
                int dr4b_port1,
                int dr4b_port2
            );

            void set_lift_state();
            void lift_functions();

    };
}