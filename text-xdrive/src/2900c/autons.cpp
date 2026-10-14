#include "main.h"

namespace autons {
// // keeping this cause its good for reference 
//     void drive_move(char direction, double speed) {
//         switch(direction) {
//             case 'F':
//                 drivetrain.set_drive_state(speed, speed, speed, speed);
//                 break;
//             case 'B':
//                 drivetrain.set_drive_state(-speed, -speed, -speed, -speed);
//                 break;
//             case 'L':
//                 drivetrain.set_drive_state(-speed, speed, speed, -speed);
//                 break;
//             case 'R':
//                 drivetrain.set_drive_state(speed, -speed, -speed, speed);
//                 break;
//         }
//     }

    void hardcoded_toggleonly() {
        drivetrain.set_drive_state(Motor_Inputs{-100, 100, 100, -100});
        pros::delay(500);
        drivetrain.set_drive_state(Motor_Inputs{0, 0, 0, 0});
    }

    void pid_test() {
        pid.move_to_pose(Pose{10, 0, 0}, 5000);
    }
}