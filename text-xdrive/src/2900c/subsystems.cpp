#include "main.h"

namespace subsystems {

    // drivetrain class

        // constructor
        Drivetrain::Drivetrain(
            int left_front_port,
            int left_back_port,
            int right_front_port,
            int right_back_port
        ):

        left_front(pros::Motor(left_front_port)),
        left_back(pros::Motor(left_back_port)),
        right_front(pros::Motor(right_front_port)),
        right_back(pros::Motor(right_back_port))
        {}

        // set drive state
        void Drivetrain::set_drive_state(float left_front_p, float left_back_p, float right_front_p, float right_back_p) {
            left_front.move(left_front_p);
            left_back.move(left_back_p);
            right_front.move(right_front_p);
            right_back.move(right_back_p);
        }

        void Drivetrain::drive_functions() {

            int forward = Controller.get_analog(FORWARD);
            int turn = Controller.get_analog(TURN);
            int strafe = Controller.get_analog(STRAFE);

            int left_front_p = forward + strafe + turn;
            int left_back_p = forward - strafe + turn;
            int right_front_p = forward - strafe - turn;
            int right_back_p = forward + strafe - turn;

            // find highest of motor inputs
            int max_val = std::abs(left_front_p);

            if(std::abs(left_back_p) > max_val) {
                max_val = std::abs(left_back_p);
            }
            else if(std::abs(right_front_p) > max_val) {
                max_val = std::abs(right_front_p);
            }
            else if(std::abs(right_back_p) > max_val) {
                max_val = std::abs(right_back_p);
            }

            Controller.print(0, 0, "ajajj");
            Controller.clear();
            pros::delay(20);

            // scale 
            if (max_val > 127) {
                float scale = 127 / max_val;
                left_front_p = left_front_p * scale;
                left_back_p = left_back_p * scale;
                right_front_p = right_front_p * scale;
                right_back_p = right_back_p * scale;
            }

            // set the drivetrain power state
            set_drive_state(
                left_front_p,
                left_back_p,
                right_front_p,
                right_back_p
            );
        }

    // --------------------------------------------------------------------------------

    // claw class
        
        // constructor
        Claw::Claw(int claw_port):

        claw_motor(pros::Motor(claw_port))
        {}

        // set claw state
        void Claw::set_claw_state(bool open) {
            if(open) { // need to tweak this variable for how many degrees it
                // is when opened fully
                claw_motor.move_absolute(-90, claw_velocity);
            }

            else { // move claw to where it was at initialization
                claw_motor.move_absolute(0, claw_velocity);
            }
        }

        void Claw::claw_functions() {
            // close claw
            if(Controller.get_digital(CLAWCLOSE)) {
                open = false;
            }

            // open claw
            else if(Controller.get_digital(CLAWOPEN)) {
                open = true;
		    }

            set_claw_state(open);
        }

}