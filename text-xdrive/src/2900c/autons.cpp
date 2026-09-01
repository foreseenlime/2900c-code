#include "main.h"

namespace autons {

    void drive_move(char direction, double speed) {
        switch(direction) {
            case 'F':
                drivetrain.set_drive_state(speed, speed, speed, speed);
            case 'B':
                drivetrain.set_drive_state(-speed, -speed, -speed, -speed);
            case 'L':
                drivetrain.set_drive_state(-speed, speed, speed, -speed);
            case 'R':
                drivetrain.set_drive_state(speed, -speed, -speed, speed);
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
        drive_move('L', 127);
        pros::delay(30);
        drive_stop();
        pros::delay(10);
        drive_move('R', 100);
        pros::delay(20);
        drive_stop();
    }

    void score_preload() {
        drive_move('R', 50);
        pros::delay(50);
        drive_stop();
    }
}