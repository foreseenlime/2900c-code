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

            float forward = Controller.get_analog(FORWARD);
            float turn = Controller.get_analog(TURN);
            float strafe = Controller.get_analog(STRAFE);

            float left_front_p = forward + strafe + turn;
            float left_back_p = forward - strafe + turn;
            float right_front_p = forward - strafe - turn;
            float right_back_p = forward + strafe - turn;

            // find highest of motor inputs
            float max_val = std::abs(left_front_p);

            if(std::abs(left_back_p) > max_val) {
                max_val = std::abs(left_back_p);
            }
            else if(std::abs(right_front_p) > max_val) {
                max_val = std::abs(right_front_p);
            }
            else if(std::abs(right_back_p) > max_val) {
                max_val = std::abs(right_back_p);
            }

            // scale to the valid motor range
            if (max_val > 127.0f) {
                float scale = 127.0f / max_val;
                left_front_p *= scale;
                left_back_p *= scale;
                right_front_p *= scale;
                right_back_p *= scale;
            }

            left_front_p = left_front_p;
            left_back_p = left_back_p;
            right_front_p = right_front_p;
            right_back_p = right_back_p;

            // set the drivetrain power state
            set_drive_state(
                (left_front_p),
                (left_back_p),
                (right_front_p),
                (right_back_p)
            );

            // set brake mode to hold for koth, makes us harder to push
            if(Controller.get_digital(DRIVEHOLD)) {
                set_brake_mode(MOTOR_BRAKE_HOLD);
            }

            // set brake mode to coast, mostly just in case driver accidentily
            // sets brake mode to hold when we dont want to
            else if(Controller.get_digital(DRIVECOAST)) {
                set_brake_mode(MOTOR_BRAKE_COAST);
            }
        }

        void Drivetrain::set_brake_mode(enum pros::motor_brake_mode_e brake_mode) {
            left_front.set_brake_mode(brake_mode);
            left_back.set_brake_mode(brake_mode);

            right_front.set_brake_mode(brake_mode);
            right_back.set_brake_mode(brake_mode);
        }

    // --------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------

    // dr4b class

        // constructor
        Lift::Lift(int dr4b_port1, int dr4b_port2,
        int claw_port, char claw_piston_port):

        dr4b_motor1(pros::Motor(dr4b_port1, pros::MotorGearset::red)),
        dr4b_motor2(pros::Motor(dr4b_port2, pros::MotorGearset::red)),

        dr4b(pros::MotorGroup({dr4b_motor1})),

        claw_motor(pros::Motor(claw_port)),
        claw_piston(pros::adi::DigitalOut(claw_piston_port))
        {
            dr4b.append(dr4b_motor2);

            dr4b_motor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            dr4b_motor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }

        void Lift::set_lift_state(double lift_voltage, double claw_voltage, bool down) {
            dr4b.move_voltage(floor(lift_voltage));
            claw_motor.move_voltage(floor(claw_voltage));
            claw_piston.set_value(down);
        }

        void Lift::lift_functions() {
            // move dr4b down
            if(Controller.get_digital(LIFTDOWN)) {
                dr4b_voltage = -12000;
            }

            // move dr4b up
            else if(Controller.get_digital(LIFTUP)) {
                dr4b_voltage = 12000;
		    }

            // stop lift
            // will this actually brake it?? idk if it doesn't, add a statement in the set
            // lift state function that checks if its 0 and brake() it
            else {
                dr4b_voltage = 0;
            }

// ---------claw functions-----------------------------------------------------------------
            // close claw
            if(Controller.get_digital(CLAWCLOSE)) {
                claw_voltage = -11000;
            }

            // need to tweak this variable for how many degrees it
            // is when opened fully
            // open claw
            else if(Controller.get_digital(CLAWOPEN)) {
                claw_voltage = 11000;
		    }

            else {
                claw_voltage = 0;
            }

            // toggle claw downwards
            if(Controller.get_digital(CLAWDOWN)) {
                down = true;
            }

            else if(Controller.get_digital(CLAWUP)) {
                down = false;
            }

            set_lift_state(dr4b_voltage, claw_voltage, down);
        }

}