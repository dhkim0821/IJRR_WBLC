#ifndef NAO_DEFINITION
#define NAO_DEFINITION

namespace nao_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    // Left Leg
    constexpr int  LHipYawPitch = 6;
    constexpr int  LHipRoll = 7;
    constexpr int  LHipPitch = 8;
    constexpr int  LKneePitch = 9;
    constexpr int  LAnklePitch = 10;
    constexpr int  LAnkleRoll = 11;

    // Left Arm
    constexpr int  LShoulderPitch = 12;
    constexpr int  LShoulderRoll = 13; 

    // Right Leg
    constexpr int  RHipYawPitch = 14; 
    constexpr int  RHipRoll = 15; 
    constexpr int  RHipPitch = 16; 
    constexpr int  RKneePitch = 17; 
    constexpr int  RAnklePitch = 18; 
    constexpr int  RAnkleRoll = 19; 

    // Right Arm
    constexpr int  RShoulderPitch = 20; 
    constexpr int  RShoulderRoll = 21; 

    constexpr int  virtual_Rw = 22;

    // Fixed Joint
    //joint 'HeadPitch' child link 'Head' type = 6
    //joint 'LFoot/FSR/FrontLeft_sensor_fixedjoint' child link 'LFsrFL_frame' type = 6
    //joint 'LFoot/FSR/FrontRight_sensor_fixedjoint' child link 'LFsrFR_frame' type = 6
    //joint 'LFoot/FSR/RearLeft_sensor_fixedjoint' child link 'LFsrRL_frame' type = 6
    //joint 'LFoot/FSR/RearRight_sensor_fixedjoint' child link 'LFsrRR_frame' type = 6
    //joint 'LElbowYaw' child link 'LElbow' type = 6
    //joint 'LElbowRoll' child link 'LForeArm' type = 6
    //joint 'LWristYaw' child link 'l_wrist' type = 6
    //joint 'LFinger11' child link 'LFinger11_link' type = 6
    //joint 'LFinger12' child link 'LFinger12_link' type = 6
    //joint 'LFinger13' child link 'LFinger13_link' type = 6
    //joint 'LFinger21' child link 'LFinger21_link' type = 6
    //joint 'LFinger22' child link 'LFinger22_link' type = 6
    //joint 'LFinger23' child link 'LFinger23_link' type = 6
    //joint 'LHand' child link 'l_gripper' type = 6
    //joint 'LThumb1' child link 'LThumb1_link' type = 6
    //joint 'LThumb2' child link 'LThumb2_link' type = 6
    //joint 'RFoot/FSR/FrontLeft_sensor_fixedjoint' child link 'RFsrFL_frame' type = 6
    //joint 'RFoot/FSR/FrontRight_sensor_fixedjoint' child link 'RFsrFR_frame' type = 6
    //joint 'RFoot/FSR/RearLeft_sensor_fixedjoint' child link 'RFsrRL_frame' type = 6
    //joint 'RFoot/FSR/RearRight_sensor_fixedjoint' child link 'RFsrRR_frame' type = 6
    //joint 'RElbowYaw' child link 'RElbow' type = 6
    //joint 'RElbowRoll' child link 'RForeArm' type = 6
    //joint 'RWristYaw' child link 'r_wrist' type = 6
    //joint 'RFinger11' child link 'RFinger11_link' type = 6
    //joint 'RFinger12' child link 'RFinger12_link' type = 6
    //joint 'RFinger13' child link 'RFinger13_link' type = 6
    //joint 'RFinger21' child link 'RFinger21_link' type = 6
    //joint 'RFinger22' child link 'RFinger22_link' type = 6
    //joint 'RFinger23' child link 'RFinger23_link' type = 6
    //joint 'RHand' child link 'r_gripper' type = 6
    //joint 'RThumb1' child link 'RThumb1_link' type = 6
    //joint 'RThumb2' child link 'RThumb2_link' type = 6
}

namespace nao{
    // Simple version
    constexpr int num_q = 23;
    constexpr int num_qdot = 22;
    constexpr int num_act_joint = 16;// 6 x2 + 2 x2

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;

    constexpr int num_leg_joint = 6;
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
