#ifndef ATLAS_DEFINITION
#define ATLAS_DEFINITION

namespace atlas_link{
    constexpr int pelvis = 0;
    constexpr int torso = 1;
    constexpr int rightHand = 4;
    constexpr int leftHand = 5;
    //constexpr int head = 6;
    constexpr int rightFoot = 7;
    constexpr int leftFoot = 8;
}

namespace atlas_joint{
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    constexpr int back_bkz = 6;
    constexpr int back_bky = 7;
    constexpr int back_bkx = 8;
   
    constexpr int l_arm_shy = 9;
    constexpr int l_arm_shx = 10;
    constexpr int l_arm_ely = 11;
    constexpr int l_arm_elx = 12;
    constexpr int l_arm_wry = 13;
    constexpr int l_arm_wrx = 14;
      
    constexpr int r_arm_shy = 15;
    constexpr int r_arm_shx = 16;
    constexpr int r_arm_ely = 17;
    constexpr int r_arm_elx = 18;
    constexpr int r_arm_wry = 19;
    constexpr int r_arm_wrx = 20;

    constexpr int l_leg_hpz = 21;
    constexpr int l_leg_hpx = 22;
    constexpr int l_leg_hpy = 23;
    constexpr int l_leg_kny = 24;
    constexpr int l_leg_aky = 25;
    constexpr int l_leg_akx = 26;

    constexpr int r_leg_hpz = 27;
    constexpr int r_leg_hpx = 28;
    constexpr int r_leg_hpy = 29;
    constexpr int r_leg_kny = 30;
    constexpr int r_leg_aky = 31;
    constexpr int r_leg_akx = 32;

    constexpr int  virtual_Rw = 33;
}

namespace atlas{
    // Simple version
    constexpr int num_q = 34;
    constexpr int num_qdot = 33;
    constexpr int num_act_joint = 27;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;
    constexpr int num_leg_joint = 6;
    constexpr int upper_body_start_jidx = atlas_joint::l_arm_shy;
    constexpr int num_upper_joint = 12;
};
#endif
