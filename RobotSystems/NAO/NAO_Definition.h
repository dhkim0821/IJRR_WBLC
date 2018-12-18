#ifndef NAO_DEFINITION
#define NAO_DEFINITION


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
