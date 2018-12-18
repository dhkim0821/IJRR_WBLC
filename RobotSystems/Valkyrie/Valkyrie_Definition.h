#ifndef VALKYRIE_DEFINITION
#define VALKYRIE_DEFINITION

namespace valkyrie_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    constexpr int  leftHipYaw = 6; // 1st joint
    constexpr int  leftHipRoll = 7;
    constexpr int  leftHipPitch = 8;
    constexpr int  leftKneePitch = 9;
    constexpr int  leftAnklePitch = 10;
    constexpr int  leftAnkleRoll = 11;

    constexpr int  rightHipYaw = 12;
    constexpr int  rightHipRoll = 13;
    constexpr int  rightHipPitch = 14;
    constexpr int  rightKneePitch = 15;
    constexpr int  rightAnklePitch = 16;
    constexpr int  rightAnkleRoll = 17;

    constexpr int  torsoYaw = 18;
    constexpr int  torsoPitch = 19;
    constexpr int  torsoRoll = 20;

    constexpr int  leftShoulderPitch = 21;
    constexpr int  leftShoulderRoll = 22;
    constexpr int  leftShoulderYaw = 23;
    constexpr int  leftElbowPitch = 24;
    constexpr int  leftForearmYaw = 25;

    constexpr int  lowerNeckPitch = 26;
    constexpr int  neckYaw = 27;
    constexpr int  upperNeckPitch = 28;

    constexpr int  rightShoulderPitch = 29;
    constexpr int  rightShoulderRoll = 30;
    constexpr int  rightShoulderYaw = 31;
    constexpr int  rightElbowPitch = 32;
    constexpr int  rightForearmYaw = 33;

    constexpr int  virtual_Rw = 34;
}

namespace valkyrie{
    // Simple version
    constexpr int num_q = 35;
    constexpr int num_qdot = 34;
    constexpr int num_act_joint = 28;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;
    constexpr int num_leg_joint = 6;
    constexpr int upper_body_start_jidx = valkyrie_joint::leftShoulderPitch;
    constexpr int num_upper_joint = valkyrie_joint::virtual_Rw - valkyrie_joint::leftShoulderPitch;
};

namespace valkyrie_link{
    constexpr int pelvis = 0;
    constexpr int torso = 1;
    constexpr int rightCOP_Frame = 2;
    constexpr int leftCOP_Frame = 3;
    constexpr int rightPalm = 4;
    constexpr int leftPalm = 5;
    constexpr int head = 6;
    constexpr int rightFoot = 7;
    constexpr int leftFoot = 8;
}

#endif
