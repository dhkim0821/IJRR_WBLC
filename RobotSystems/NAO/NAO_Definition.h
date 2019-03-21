#ifndef NAO_DEFINITION
#define NAO_DEFINITION

namespace nao_joint{
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



namespace nao{
    //constexpr int num_q = 31;
    //constexpr int num_qdot = 30;
    //constexpr int num_act_joint = 24;//2 + 6 x2 + 5 x2

    // Simple version
    constexpr int num_q = 23;
    constexpr int num_qdot = 22;
    constexpr int num_act_joint = 16;// 6 x2 + 2 x2

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;

    constexpr int num_leg_joint = 6;
    constexpr int upper_body_start_jidx = nao_joint::l_arm_shy;
    constexpr int num_upper_joint = 12;
};

namespace nao_link{
    constexpr int torso = 0;
    constexpr int LPelvis = 1;

    constexpr int r_ankle = 2;
    constexpr int l_ankle = 3;
    
    constexpr int RFsrFL_frame = 5;
    constexpr int RFsrFR_frame = 6;
    constexpr int RFsrRL_frame = 7;
    constexpr int RFsrRR_frame = 8;
    
    constexpr int LFsrFL_frame = 9;
    constexpr int LFsrFR_frame = 10;
    constexpr int LFsrRL_frame = 11;
    constexpr int LFsrRR_frame = 12;
    
    constexpr int l_sole = 13;
    constexpr int r_sole = 14;
}

#endif
