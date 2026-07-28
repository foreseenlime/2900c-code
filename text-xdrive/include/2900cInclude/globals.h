#pragma once

// motors:

    // drivetrain motors
        #define LEFTFRONT 1
        #define LEFTBACK 2

        #define RIGHTFRONT -3
        #define RIGHTBACK -4

    // dr4b motors
        #define LIFTLEFT -20
        #define LIFTRIGHT 19

    // claw motors
        #define CLAW 11

// controls:

    //drive
        #define FORWARD ANALOG_LEFT_Y
        #define TURN ANALOG_RIGHT_X
        #define STRAFE ANALOG_LEFT_X

    // dr4b
        #define LIFTDOWN DIGITAL_R1
        #define LIFTUP DIGITAL_R2

    // claw
        #define CLAWCLOSE DIGITAL_L1
        #define CLAWOPEN DIGITAL_L2