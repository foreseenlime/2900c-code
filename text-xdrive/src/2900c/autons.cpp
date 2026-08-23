#include "main.h"

namespace autons {

    void drive(char direction, double speed) {
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

    void rotate(char direction, double speed) {
        switch(direction) {
            case 'C':
                // if rotation is clockwise: forward = 0, strafe = 0, turn = 127
                    // lf = 0+0+127 = 127
                    // lb = 0-0+127 = 127
                    // rf = 0-0-127 = -127
                    // rb = 0+0-127 = -127
                drivetrain.set_drive_state(speed, speed, -speed, -speed);
            case 'A':
                drivetrain.set_drive_state(-speed, -speed, speed, speed);
        }
    }

    void stop() {drivetrain.set_drive_state(0, 0, 0, 0);}

    void toggle_only() {
        // drive left
        // if movement is left: forward = 0, strafe = -127, turn = 0
            // lf = 0-127+0 = -127
            // lb = 0+127+0 = 127
            // rf = 0+127-0 = 127
            // rb = 0-127-0 = -127
        drive('L', 127);
        pros::delay(30);
        stop();
        pros::delay(10);
        drive('R', 100);
        pros::delay(20);
        stop();
    }
}