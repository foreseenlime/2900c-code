#include "main.h"

namespace autons {

    void drive_move(char direction, double speed) {
        switch(direction) {
            case 'F':
                drivetrain.set_drive_state(speed, speed, speed, speed);
                break;
            case 'B':
                drivetrain.set_drive_state(-speed, -speed, -speed, -speed);
                break;
            case 'L':
                drivetrain.set_drive_state(-speed, speed, speed, -speed);
                break;
            case 'R':
                drivetrain.set_drive_state(speed, -speed, -speed, speed);
                break;
        }
    }

    void drive_rotate(char direction, double speed) {
        switch(direction) {
            case 'C':
                drivetrain.set_drive_state(speed, speed, -speed, -speed);
            case 'A':
                drivetrain.set_drive_state(-speed, -speed, speed, speed);
        }
    }

    void drive_stop() {drivetrain.set_drive_state(0, 0, 0, 0);}

    void toggle_only() {
        lift.set_lift_state(-100, 0, false);
        pros::delay(600);
        lift.set_lift_state(0, 0, false);

        drive_move('L', 100);

        pros::delay(300);
        drive_stop();
        pros::delay(400);

        drive_move('R', 100);

        pros::delay(550);
        drive_stop();
        pros::delay(500);

        drive_move('L', 100);

        pros::delay(600);
        drive_stop();
        pros::delay(500);

        drive_move('R', 100);

        pros::delay(550);
        drive_stop();
        pros::delay(500);

        drive_move('L', 100);

        pros::delay(600);
        drive_stop();
        pros::delay(500);

        drive_move('R', 100);

        pros::delay(500);
        drive_stop();
    }

    // void score_preload() {
    //     drive_move('R', 50);
    //     pros::delay(50);
    //     drive_stop();
    // }
}