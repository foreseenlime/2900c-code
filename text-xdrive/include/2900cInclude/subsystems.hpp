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
                std::int8_t left_front_port,
                std::int8_t left_back_port, 
                std::int8_t right_front_port,
                std::int8_t right_back_port
            );

            /**
             * @brief Sets the voltage for each motor in the drivetrain (-127 - 127)
             * 
             * @param left_front_p Left front motor voltage
             * @param left_back_p Left back motor voltage
             * @param right_front_p Right front motor voltage
             * @param right_back_p Right back motor voltage
             */
            void set_drive_state(float left_front_p, float left_back_p, float right_front_p, float right_back_p);
            void drive_functions();
            void set_brake_mode(enum pros::motor_brake_mode_e brake_mode);
    };

    class Lift {

        private:
            pros::Motor claw_motor;
            pros::adi::DigitalOut claw_piston;
            double claw_speed;
            bool down = true;

            pros::Motor dr4b_motor1;
            pros::Motor dr4b_motor2;
            pros::MotorGroup dr4b;

            double dr4b_speed;

        public:
            // constructor
            Lift( // changing the datatype of all the port variables made it possible to use them in motor groups as intended!!!!!!!!!!!!!! include in design book!!!!!!!!!
                std::int8_t dr4b_port1,
                std::int8_t dr4b_port2,
                std::int8_t claw_port,
                char claw_piston_port
            );

            /**
             * @brief Sets the voltage + piston state of the claw and dr4b
             * 
             * @param lift_speed Speed that the lift motors move (-100 - 100)
             * @param claw_speed Speed that the claw motor moves (-200 - 200)
             * @param down State of piston that puts the claw down/up (true, false)
             */
            void set_lift_state(double lift_speed, double claw_speed, bool down);
            void lift_functions();

    };
}