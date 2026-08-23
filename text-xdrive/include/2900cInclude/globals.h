#pragma once

// motors:

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

// controls:

    //drive
        #define FORWARD ANALOG_LEFT_Y
        #define TURN ANALOG_RIGHT_X
        #define STRAFE ANALOG_LEFT_X

        #define DRIVEHOLD DIGITAL_Y
        #define DRIVECOAST DIGITAL_B

    // dr4b
        #define LIFTDOWN DIGITAL_R1
        #define LIFTUP DIGITAL_R2

    // claw
        #define CLAWCLOSE DIGITAL_L1
        #define CLAWOPEN DIGITAL_L2

        #define CLAWDOWN DIGITAL_DOWN
        #define CLAWUP DIGITAL_RIGHT