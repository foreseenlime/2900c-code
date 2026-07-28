#include "main.h"

pros::Motor left_front(LEFTFRONT);
pros::Motor left_back(LEFTBACK);
pros::Motor right_front(RIGHTFRONT);
pros::Motor right_back(RIGHTBACK);

pros::Motor dr4b1(LIFTLEFT, pros::MotorGearset::red);
pros::Motor dr4b2(LIFTRIGHT, pros::MotorGearset::red);

pros::MotorGroup dr4b(dr4b1);

pros::Motor claw(CLAW, pros::MotorGearset::green);

// DO THIS FOR DRIVETRAIN: https://www.vexforum.com/t/v5-x-drive-pros-code/62326


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	dr4b.append(dr4b2);

	// left_front.set_reversed(false);
	// left_back.set_reversed(false);
	// right_front.set_reversed(true);
	// right_back.set_reversed(true);

	dr4b1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	dr4b2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	while (true) {

		int forward = master.get_analog(FORWARD);
		int turn    = master.get_analog(TURN);
		int strafe  = master.get_analog(STRAFE);

		int lf = forward + strafe + turn;
		int lb = forward - strafe + turn;
		int rf = forward - strafe - turn;
		int rb = forward + strafe - turn;

		// find highest of motor inputs
		int max_val = std::abs(lf);

		if(std::abs(lb) > max_val) {
			max_val = std::abs(lb);
		}
		else if(std::abs(rf) > max_val) {
			max_val = std::abs(rf);
		}
		else if(std::abs(rb) > max_val) {
			max_val = std::abs(rb);
		}

		// scale 
		if (max_val > 127) {
			float scale = 127 / max_val;
			lf = lf * scale;
			lb = lb * scale;
			rf = rf * scale;
			rb = rb * scale;
		}

		// move motors
		left_front.move(lf);
		left_back.move(lb);
		right_front.move(rf);
		right_back.move(rb);

		// move dr4b down
		if(master.get_digital(LIFTDOWN)) {
			dr4b.move_velocity(-100);
		}
		// move dr4b up
		else if(master.get_digital(LIFTUP)) {
			dr4b.move_velocity(100);
		}
		else {
			dr4b.brake();
		}

		// close claw
		if(master.get_digital(CLAWCLOSE)) {
			claw.move_velocity(200);
		}
		// open claw
		else if(master.get_digital(CLAWOPEN)) {
			claw.move_velocity(-200);
		}
		else {
			claw.brake();
		}

		pros::delay(20);
	}
}