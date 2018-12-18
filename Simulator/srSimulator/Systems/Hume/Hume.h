#ifndef _HUME_
#define _HUME_

#include "srDyn/srSpace.h"
#include "Ground.h"
#include <vector>
#include <srSysGenerator/SystemGenerator.h>

#define SIM_NUM_RJOINT	6
#define SIM_NUM_PASSIVE_R   3
#define SIM_NUM_PASSIVE_P   3  //R+P = num of passive
#define SIM_NUM_PASSIVE     SIM_NUM_PASSIVE_P + SIM_NUM_PASSIVE_R
#define SIM_NUM_JOINT       SIM_NUM_RJOINT    + SIM_NUM_PASSIVE

////////////////////////////
#define SIM_NUM_LINK	9 + 6 + 1 //Link + Passive + Boom
#define SIM_NUM_WJOINT	3
#define SIM_FOOT_RADIUS     0.025
#define SIM_HUME_TILTING 0.25
#define BOOM_height 0.07

#define SIM_NUM_LED 7 + 10

class HumeID{
public:
    enum JointID{
        SIM_J_RIGHT_HIP1 = 3,	SIM_J_RIGHT_HIP2 = 4,       SIM_J_RIGHT_KNEE = 5,
        SIM_J_LEFT_HIP1 = 6,	SIM_J_LEFT_HIP2 = 7,	    SIM_J_LEFT_KNEE = 8

    };
    // Px, Py, Pz, Rz, Ry, Rx,
    // R_Abduction, R_HIP, R_Knee, L_Abduction, L_HIP, L_Knee
    enum PassiveJoint{
        SIM_X = 0,  SIM_Y = 1,  SIM_Z = 2, //Prismatic
        SIM_Rx = 2, SIM_Ry = 1, SIM_Rz = 0  //Rotation
    };
    
    enum LinkID{
        SIM_HIP = 0,	        SIM_LEFT_HIP_SIDE = 1,	SIM_LEFT_THIGH = 2, SIM_LEFT_CALF = 3,
        SIM_RIGHT_HIP_SIDE = 4,	SIM_RIGHT_THIGH = 5,	SIM_RIGHT_CALF = 6,
        SIM_LFOOT = 7,          SIM_RFOOT = 8,          
        SIM_Base = 9,           SIM_L_Y = 10,           SIM_L_Z = 11,
        SIM_L_Rz = 12,          SIM_L_Ry = 13,          SIM_L_Rx = 14,
        SIM_Boom = 15
    };
    
    
    enum WJointID
    {
        WJ_LFOOT = 0, WJ_RFOOT = 1, WJ_Boom = 2
    };
};


/* class Hume: public srSystem */
class Hume: public SystemGenerator
{
public:
    Hume(const Vec3 & location, srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type);
    virtual ~Hume(){}
    void SetConfiguration(const std::vector<double>& _conf);

public:
    double starting_height_;
    double m_hip_size_half;
    double m_leng_calf;

    double thigh_rad_;
    double hume_tilting_;
    srRevoluteJoint	m_Rjoint[SIM_NUM_RJOINT + SIM_NUM_PASSIVE_R];
    srPrismaticJoint    m_Pjoint[SIM_NUM_PASSIVE_P];
    srRevoluteState     *Full_Joint_State_[SIM_NUM_JOINT];
    
    srLink		m_Link[SIM_NUM_LINK];
    srCollision		m_Collision[SIM_NUM_LINK];
    srWeldJoint		m_Wjoint[SIM_NUM_WJOINT];
    srWeldJoint Wjoint_LED_[SIM_NUM_LED];
    srLink LED_[SIM_NUM_LED];

private:
    void _AssembleModel(const Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type);
    void _SetInertia();
    void _SetLinkShape();
    void _SetPassiveJoint(srJoint::ACTTYPE joint_type);
    void _SetInitialConf();
    void _Foot_Assemble();
    void _SetLED();

private:
    double R1axis_titling; 
    double R2axis_tilting; 
    double L1axis_tilting; 
    double L2axis_tilting; 

    ///////////   MASS   ///////////////
    double M_abduction; 
    double M_Thigh    ;     
    double M_body     ;      
    double M_calf     ;      
                                       
    //////NK LENGTH  //////////        
    const double ly6;         
    const double lz7;       
    const double lx8;         
    const double lz8;          
                                       
    // //////M LENGTH  //////////         
    const double px6;     
    const double py6;     
    const double pz6;     
    const double px7;     
    const double py7;     
    const double pz7;     
    const double px8;     
    const double py8;     
    const double pz8;     
    const double px9;     
    const double pz9;     
};


#endif
