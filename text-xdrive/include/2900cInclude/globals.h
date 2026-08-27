#pragma once

// ports:

    // drivetrain ports
        #define LEFTFRONT 1
        #define LEFTBACK 2

        #define RIGHTFRONT -3
        #define RIGHTBACK -4

    // dr4b ports
        #define LIFTLEFT -20
        #define LIFTRIGHT 19

    // claw ports
        #define CLAW_MOTOR 11
        #define CLAW_PISTON 'A'

// odom:

    // ports:
        #define IMU 10
        #define HORIZONTAL 9
        #define VERTICAL 8

    // rotation sensor offsets (unit in inches)
        #define HORIZONTAL_OFFSET 0
        #define VERTICAL_OFFSET 0

    // drivetrain (unit in inches)
        // #define TRACKWIDTH 18 // needs measuring
        // #define WHEELBASE 18 // needs measuring
        #define CIRCUMFERENCE 4
        #define TRACKER_CIRCUMFERENCE 2

// controls:

    // drive
        #define FORWARD ANALOG_LEFT_Y
        #define TURN ANALOG_RIGHT_X
        #define STRAFE ANALOG_LEFT_X

        #define DRIVEHOLD DIGITAL_Y
        #define DRIVECOAST DIGITAL_B

        #define IMURESET DIGITAL_LEFT

    // dr4b
        #define LIFTDOWN DIGITAL_R1
        #define LIFTUP DIGITAL_R2

    // claw
        #define CLAWCLOSE DIGITAL_L1
        #define CLAWOPEN DIGITAL_L2

        #define CLAWDOWN DIGITAL_DOWN
        #define CLAWUP DIGITAL_RIGHT