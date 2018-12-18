#ifndef MERCURY_DEFINITION
#define MERCURY_DEFINITION

namespace mercury{
    constexpr int num_q = 13;
    constexpr int num_qdot = 12;
    constexpr int num_act_joint = 6;
    constexpr int num_virtual = 6;
    constexpr double servo_rate = 1.0/1500.0;
    //constexpr double servo_rate = 1.0/1000.0;
};

namespace mercury_link{
    constexpr int body = 0;
    constexpr int rightFoot = 1;
    constexpr int leftFoot = 2;
    constexpr int imu = 3;

    constexpr int LED_BODY_0 = 20;
    constexpr int LED_BODY_1 = 21;
    constexpr int LED_BODY_2 = 22;

    constexpr int LED_RLEG_0 = 23;
    constexpr int LED_RLEG_1 = 24;
    constexpr int LED_RLEG_2 = 25;
    constexpr int LED_RLEG_3 = 26;
    constexpr int LED_RLEG_4 = 27;

    constexpr int LED_LLEG_0 = 28;
    constexpr int LED_LLEG_1 = 29;
    constexpr int LED_LLEG_2 = 30;
    constexpr int LED_LLEG_3 = 31;
    constexpr int LED_LLEG_4 = 32;
};

namespace mercury_joint{
    constexpr int virtual_X = 0;
    constexpr int virtual_Y = 1;
    constexpr int virtual_Z = 2;
    constexpr int virtual_Rx = 3;
    constexpr int virtual_Ry = 4;
    constexpr int virtual_Rz = 5;

    constexpr int rightAbduction = 6;
    constexpr int rightHip = 7;
    constexpr int rightKnee = 8;
    
    constexpr int leftAbduction = 9;
    constexpr int leftHip = 10;
    constexpr int leftKnee = 11;
   
    constexpr int virtual_Rw = 12;
};

namespace mercury_trj_type{
    constexpr int sinusoidal = 0;
    constexpr int ramp = 1;
    constexpr int step = 2;
    constexpr int impulse = 3;
};

#endif

