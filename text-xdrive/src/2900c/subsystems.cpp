#include "main.h"

namespace subsystems {

    // drivetrain class

        // constructor
        Drivetrain::Drivetrain(
            std::int8_t left_front_port,
            std::int8_t left_back_port,
            std::int8_t right_front_port,
            std::int8_t right_back_port,
            std::int8_t imu_port,
            std::int8_t horizontal_port,
            std::int8_t vertical_port
        ):

        left_front(pros::Motor(left_front_port)),
        left_back(pros::Motor(left_back_port)),
        right_front(pros::Motor(right_front_port)),
        right_back(pros::Motor(right_back_port)),

        imu(pros::Imu(imu_port)),
        horizontal(pros::Rotation(horizontal_port)),
        vertical(pros::Rotation(vertical_port))
        {
            horizontal.reset_position();
            vertical.reset_position();
        }

        // set drive state
        void Drivetrain::set_drive_state(float left_front_p, float left_back_p, float right_front_p, float right_back_p) {
            left_front.move(left_front_p);
            left_back.move(left_back_p);
            right_front.move(right_front_p);
            right_back.move(right_back_p);

            // if joysticks aren't being pressed, brake motors
            // this part is required to use our koth brake hold mode
            if(left_front_p == 0 && left_back_p == 0 && right_front_p == 0 && right_back_p == 0){
                left_front.brake();
                left_back.brake();
                right_front.brake();
                right_back.brake();
            }
        }

        void Drivetrain::set_brake_mode(enum pros::motor_brake_mode_e brake_mode) {
            left_front.set_brake_mode(brake_mode);
            left_back.set_brake_mode(brake_mode);

            right_front.set_brake_mode(brake_mode);
            right_back.set_brake_mode(brake_mode);
        }

        void Drivetrain::drive_functions() {

            double forward = linear_to_squared(Controller.get_analog(FORWARD), 127);
            double turn = linear_to_squared(Controller.get_analog(TURN), 127);
            double strafe = linear_to_squared(Controller.get_analog(STRAFE), 127);

            // START OF CODE THAT NEEDS UNDERSTANDING + NOTEBOOKING ------------------------------------------------------------------------

            // same as projectile motion!!
            // magnitude = velocity
            // analog_theta = angle of velocity
            // vertical velocity = forwards
            // horizontal velocity = strafe

            // first: convert forward + backward joystick values (vertical + horizontal velocities) to a single magnitude + direction (velocity at an angle)

            // gets the distance that the joystick is being moved
            double magnitude = sqrt(pow(forward, 2) + pow(strafe, 2));
            // gets the angle that the joystick is being pushed (in radians)
            double analog_theta = atan2(strafe, forward);

            // get heading in radians
            double heading = get_rad(imu.get_heading());
            // get new angle for forwards + strafe
            // still kinda need to figure out how this works, more of an intermediary step rn
            double new_dir = analog_theta + heading;

            // convert magnitude (velocity) back to forward + backward (vertical + horizontal) components using the new theta value
            forward = magnitude * sin(new_dir); // multiply by -1 cause for some reason the movement is backwards
            strafe = magnitude * cos(new_dir);

            // then continue to conversion to motor values :)

            // END OF CODE THAT NEEDS UNDERSTANDING + NOTEBOOKING --------------------------------------------------------------------

            double left_front_p = forward + strafe + turn;
            double left_back_p = forward - strafe + turn;
            double right_front_p = forward - strafe - turn;
            double right_back_p = forward + strafe - turn;

            // find highest of motor inputs
            double max_val = std::abs(left_front_p);
            if(std::abs(left_back_p) > max_val) {max_val = std::abs(left_back_p);}
            else if(std::abs(right_front_p) > max_val) {max_val = std::abs(right_front_p);}
            else if(std::abs(right_back_p) > max_val) {max_val = std::abs(right_back_p);}

            // scale to the valid motor range
            if (max_val > 127) {
                double scale = 127 / max_val;
                left_front_p *= scale;
                left_back_p *= scale;
                right_front_p *= scale;
                right_back_p *= scale;
            }

            // set the drivetrain power state
            set_drive_state(
                floor(left_front_p), 
                floor(left_back_p), 
                floor(right_front_p), 
                floor(right_back_p)
            );

            // set brake mode to hold for koth, makes us harder to push
            if(Controller.get_digital(DRIVEHOLD)) {set_brake_mode(MOTOR_BRAKE_HOLD);}

            // set brake mode to coast, mostly just in case driver accidentily
            // sets brake mode to hold when we dont want to
            else if(Controller.get_digital(DRIVECOAST)) {set_brake_mode(MOTOR_BRAKE_COAST);}

            // reset imu if needed
            if(Controller.get_digital(IMURESET)) {imu.tare_heading();}

            // print all motor temps to brain console
            console.printf(
                "LF: %.0lf, LB: %.0lf, RF: %.0lf, RB: %.0lf \n", 
                left_front.get_temperature(), left_back.get_temperature(), 
                right_front.get_temperature(), right_back.get_temperature()
            );
        }

    // --------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------

    // dr4b class

        // constructor
        Lift::Lift(std::int8_t dr4b_port1, std::int8_t dr4b_port2,
        std::int8_t claw_port, char claw_piston_port):

        dr4b_motor1(pros::Motor(dr4b_port1, pros::MotorGearset::red)),
        dr4b_motor2(pros::Motor(dr4b_port2, pros::MotorGearset::red)),

        dr4b(pros::MotorGroup({dr4b_port1, dr4b_port2}, pros::MotorGearset::red)),

        claw_motor(pros::Motor(claw_port)),
        claw_piston(pros::adi::DigitalOut(claw_piston_port))
        {
            dr4b_motor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            dr4b_motor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

            claw_piston.set_value(true);
        }

        void Lift::set_lift_state(double lift_speed, double claw_speed, bool down) {
            dr4b.move_velocity(floor(lift_speed));
            claw_motor.move_velocity(floor(claw_speed));
            claw_piston.set_value(down);

            // if either voltage is 0 brake them
            if(lift_speed == 0) {dr4b.brake();}
            if(claw_speed == 0) {claw_motor.brake();}
        }

        void Lift::lift_functions() {
            // move dr4b up
            if(Controller.get_digital(LIFTDOWN)) {
                dr4b_speed = -100;
                down = false;
            }

            // move dr4b down
            else if(Controller.get_digital(LIFTUP)) {dr4b_speed = 100;}

            // stop lift
            else {dr4b_speed = 0;}

            // ---------claw functions------------------------------------------------------
            // close claw
            if(Controller.get_digital(CLAWCLOSE)) {claw_speed = -180;}

            // open claw
            else if(Controller.get_digital(CLAWOPEN)) {claw_speed = 180;}

            else {claw_speed = 0;}

            // toggle claw downwards
            if(Controller.get_digital(CLAWDOWN)) {down = true;}

            else if(Controller.get_digital(CLAWUP)) {down = false;}

            set_lift_state(dr4b_speed, claw_speed, down); 
            
            // print all temps to brain screen
            console.printf(
                "dr4b1: %.0lf, dr4b2: %.0lf, claw: %.0lf",
                dr4b_motor1.get_temperature(), dr4b_motor2.get_temperature(), claw_motor.get_temperature()
            );
        }

}