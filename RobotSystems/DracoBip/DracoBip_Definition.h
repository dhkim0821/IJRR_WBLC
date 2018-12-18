#ifndef DRACO_BIPED_DEFINITION
#define DRACO_BIPED_DEFINITION

namespace dracobip_link{
    constexpr int torso = 0;

    constexpr int lHipYaw = 1;
    constexpr int lHipRoll = 2;
    constexpr int lHipPitch = 3;
    constexpr int lKnee = 4;
    constexpr int lAnkle = 5;

    constexpr int rHipYaw = 6;
    constexpr int rHipRoll = 7;
    constexpr int rHipPitch = 8;
    constexpr int rKnee = 9;
    constexpr int rAnkle = 10;

    constexpr int LED_BODY_0 = 20;
    constexpr int LED_BODY_1 = 21;
    constexpr int LED_BODY_2 = 22;
}

namespace dracobip_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    constexpr int lHipYaw = 6;
    constexpr int lHipRoll = 7;
    constexpr int lHipPitch = 8;
    constexpr int lKnee = 9;
    constexpr int lAnkle = 10;
    
    constexpr int rHipYaw = 11;
    constexpr int rHipRoll = 12;
    constexpr int rHipPitch = 13;
    constexpr int rKnee = 14;
    constexpr int rAnkle = 15;
 
    constexpr int  virtual_Rw = 16;
}

namespace dracobip{
    // Simple version
    constexpr int num_q = 17;
    constexpr int num_qdot = 16;
    constexpr int num_act_joint = 10;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 1./1000.;
    constexpr int num_leg_joint = 5; // How many joint in each leg
};
#endif
