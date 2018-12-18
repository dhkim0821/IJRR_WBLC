#include "Hume.h"
#include <time.h>
#include <iostream>
#include <stdio.h>
#include "simulation_setting.h"
#include "Configuration.h"
#include <ControlSystem/Hume_Controller/HumeSystem.h>


extern simulation_setting * sim_setting;

Hume::Hume(const  Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type)
    :m_hip_size_half(0.15), m_leng_calf(0.44), thigh_rad_(0.07), hume_tilting_(0.1),
     ly6(0.1397  ), 
     lz7(-0.03535), 
     lx8(0.0181  ), 
     lz8(-0.4575   ),
     ///////////////////////////
     px6(0         ),         
     py6(0         ),         
     pz6(0.22524      ),      
     px7(-34.16e-3 ), 
     py7(-0.75e-3  ),  
     pz7(42.93e-3  ),  
     px8(-3.96e-3  ),  
     py8(0.18e-3   ),   
     pz8(-147.77e-3),
     px9(0         ),         
     pz9(-0.25     ),
     R1axis_titling(0.05974278), 
     R2axis_tilting(-0.05974278), 
     L1axis_tilting(0.0447249), 
     L2axis_tilting(-0.0047242)
{
    M_body      = M_BODY;
    M_abduction = M_ABDUCTION;
    M_Thigh     = M_THIGH;
    M_calf      = M_CALF;
    ////////////////////////////
    _SetInitialConf();
    _AssembleModel(location, base_link_type, joint_type);

    printf("[HUME] END of HUME assemble\n");

    for (int i(0); i< SIM_NUM_PASSIVE_P; ++i){
        Full_Joint_State_[i] = & m_Pjoint[i].m_State;
    }
    for (int i(0); i< SIM_NUM_RJOINT + SIM_NUM_PASSIVE_R; ++i){
        Full_Joint_State_[i + SIM_NUM_PASSIVE_P] = & m_Rjoint[i].m_State;
    }
}


void Hume::_AssembleModel(const Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type)
{
    double HIP_1[2] = {-50.0, 50.0};
    double HIP_2[2] = {-100.0, 100.0};
    double KNEE_LIMIT[2] = {-20.0, 200.0};
    

///////////// LEFT ////////////////////
    //Hip Joint
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_PosLimit[0] = HIP_1[0];
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_PosLimit[1] = HIP_1[1];
    
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].SetParentLink(&m_Link[HumeID::SIM_HIP]);
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].SetChildLink(&m_Link[HumeID::SIM_LEFT_HIP_SIDE]);
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, ly6 , -pz6)));
    m_Rjoint[HumeID::SIM_J_LEFT_HIP1].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF,0.0), Vec3(-px7, -py7, -pz7)));

    //Hip Side Joint
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_PosLimit[0] = HIP_2[0];
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_PosLimit[1] = HIP_2[1];
        
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].SetParentLink(&m_Link[HumeID::SIM_LEFT_HIP_SIDE]);
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].SetChildLink(&m_Link[HumeID::SIM_LEFT_THIGH]);
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].SetParentLinkFrame(EulerZYX(Vec3(L1axis_tilting, SR_PI_HALF, -SR_PI_HALF), Vec3(-px7, -py7, -pz7+lz7)));
    m_Rjoint[HumeID::SIM_J_LEFT_HIP2].SetChildLinkFrame(EulerZYX(Vec3( SR_PI_HALF, SR_PI_HALF,0.0), Vec3( -px8, -py8, -pz8)));
	
    //Knee Joint
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_PosLimit[0] = KNEE_LIMIT[0];
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_PosLimit[1] = KNEE_LIMIT[1];
    
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].SetParentLink(&m_Link[HumeID::SIM_LEFT_THIGH]);
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].SetChildLink(&m_Link[HumeID::SIM_LEFT_CALF]);
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].SetParentLinkFrame(EulerZYX(Vec3(L2axis_tilting, SR_PI_HALF, -SR_PI_HALF), Vec3( -px8 + lx8, -py8, -pz8 + lz8)));
    m_Rjoint[HumeID::SIM_J_LEFT_KNEE].SetChildLinkFrame(EulerZYX(Vec3(0.0,SR_PI_HALF, -SR_PI_HALF), Vec3( -px9, 0.0, -pz9)));

///////////// Right ////////////////////
    //Hip Joint
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_PosLimit[0] = HIP_1[0];
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_PosLimit[1] = HIP_1[1];
    
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].SetParentLink(&m_Link[HumeID::SIM_HIP]);
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].SetChildLink(&m_Link[HumeID::SIM_RIGHT_HIP_SIDE]);
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, -ly6, -pz6)));
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF,0.0), Vec3(-px7, -py7, -pz7)));
    
    //Hip Side Joint
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_PosLimit[0] = HIP_2[0];
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_PosLimit[1] = HIP_2[1];
        
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].SetParentLink(&m_Link[HumeID::SIM_RIGHT_HIP_SIDE]);
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].SetChildLink(&m_Link[HumeID::SIM_RIGHT_THIGH]);
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].SetParentLinkFrame(EulerZYX(Vec3(R1axis_titling, SR_PI_HALF, -SR_PI_HALF), Vec3(-px7, -py7, -pz7 + lz7)));
    m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].SetChildLinkFrame(EulerZYX(Vec3( SR_PI_HALF, SR_PI_HALF,0.0), Vec3( -px8, -py8, -pz8)));

    //Knee Joint
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_PosLimit[0] = KNEE_LIMIT[0];
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_PosLimit[1] = KNEE_LIMIT[1];
    
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].SetParentLink(&m_Link[HumeID::SIM_RIGHT_THIGH]);
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].SetChildLink(&m_Link[HumeID::SIM_RIGHT_CALF]);
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].SetParentLinkFrame(EulerZYX(Vec3(R2axis_tilting, SR_PI_HALF, -SR_PI_HALF), Vec3( -px8+lx8, -py8, -pz8  + lz8)));
    m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].SetChildLinkFrame(EulerZYX(Vec3(0.0,SR_PI_HALF, - SR_PI_HALF), Vec3( -px9, 0.0, -pz9)));

                                                          
    _Foot_Assemble();
    //////////////////////////////////////////////////////////////////////////
    ///////  Collision Setting
    //////////////////////////////////////////////////////////////////////////
    
    //Hip Link
    m_Collision[HumeID::SIM_HIP].GetGeomInfo().SetShape(m_Link[HumeID::SIM_HIP].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_HIP].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_HIP].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_HIP].AddCollision(&m_Collision[HumeID::SIM_HIP]);
//Left
    //Hip Side Link
    m_Collision[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().SetShape(m_Link[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_LEFT_HIP_SIDE].AddCollision(&m_Collision[HumeID::SIM_LEFT_HIP_SIDE]);
    
    //Thigh
    m_Collision[HumeID::SIM_LEFT_THIGH].GetGeomInfo().SetShape(m_Link[HumeID::SIM_LEFT_THIGH].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_LEFT_THIGH].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_LEFT_THIGH].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_LEFT_THIGH].AddCollision(&m_Collision[HumeID::SIM_LEFT_THIGH]);
    
    //Calf
    m_Collision[HumeID::SIM_LEFT_CALF].GetGeomInfo().SetShape(m_Link[HumeID::SIM_LEFT_CALF].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_LEFT_CALF].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_LEFT_CALF].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_LEFT_CALF].AddCollision(&m_Collision[HumeID::SIM_LEFT_CALF]);

//Right
	//Hip Side Link
	m_Collision[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().SetShape(m_Link[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().GetShape());
	m_Collision[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().GetDimension());
	m_Link[HumeID::SIM_RIGHT_HIP_SIDE].AddCollision(&m_Collision[HumeID::SIM_RIGHT_HIP_SIDE]);

	//Thigh
	m_Collision[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().SetShape(m_Link[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().GetShape());
	m_Collision[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().GetDimension());
	m_Link[HumeID::SIM_RIGHT_THIGH].AddCollision(&m_Collision[HumeID::SIM_RIGHT_THIGH]);

	//Calf
	m_Collision[HumeID::SIM_RIGHT_CALF].GetGeomInfo().SetShape(m_Link[HumeID::SIM_RIGHT_CALF].GetGeomInfo().GetShape());
	m_Collision[HumeID::SIM_RIGHT_CALF].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_RIGHT_CALF].GetGeomInfo().GetDimension());
	m_Link[HumeID::SIM_RIGHT_CALF].AddCollision(&m_Collision[HumeID::SIM_RIGHT_CALF]);


	//////////////////////////////////////////////////////////////////////////
	///////  Weld Joint Setting
	//////////////////////////////////////////////////////////////////////////


//	srJoint::ACTTYPE joint_type(srJoint::TORQUE);
	m_Rjoint[HumeID::SIM_J_LEFT_HIP1].SetActType(joint_type);
	m_Rjoint[HumeID::SIM_J_LEFT_HIP2].SetActType(joint_type);
	m_Rjoint[HumeID::SIM_J_LEFT_KNEE].SetActType(joint_type);

	m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].SetActType(joint_type);
	m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].SetActType(joint_type);
	m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].SetActType(joint_type);

        // if(HUME_System::GetHumeSystem()->b_sagittal_motion_){
        //     m_Rjoint[HumeID::SIM_J_LEFT_HIP1].SetActType(srJoint::HYBRID);
        //     m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].SetActType(srJoint::HYBRID);
        // }
        
        _SetLinkShape();
        _SetPassiveJoint(joint_type);
        _SetLED();
        _SetInertia();

	//Base Setting
//	m_Link[HumeID::SIM_HIP].SetFrame(EulerZYX(Vec3(0.0,HUME_TILTING,0.0),location));
        m_Link[HumeID::SIM_Base].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), location));
        this->SetBaseLink(&m_Link[HumeID::SIM_Base]);

	this->SetBaseLinkType(base_link_type);
	this->SetSelfCollision(true);
}

void Hume::SetConfiguration( const std::vector<double>& _conf )
{
	for(int i(0); i<SIM_NUM_RJOINT-1; ++i)
	{
		m_Rjoint[i].m_State.m_rValue[0] = _conf[i];
	}
	KIN_UpdateFrame_All_The_Entity();
}

void Hume::_SetLED(){
    for (int i(0); i< SIM_NUM_LED; ++i){
        LED_[i].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
        LED_[i].GetGeomInfo().SetDimension(0.02, 0.0, 0.0);
        LED_[i].GetGeomInfo().SetColor(0.9, 0.0, 0.0);
    }
    for (int i(0); i< 7; ++i){
        Wjoint_LED_[i].SetParentLink(&m_Link[HumeID::SIM_HIP]);
        Wjoint_LED_[i].SetChildLink(&LED_[i]);
    }
    Wjoint_LED_[0].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.075, 0.0, 0.2)));
    Wjoint_LED_[1].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07, 0.0, 0.2)));
    Wjoint_LED_[2].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.075, 0.0, 0.0)));
    
    Wjoint_LED_[3].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07, 0.14, 0.38-pz6))); // (X, Y, Z)
    Wjoint_LED_[4].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.07, 0.14, 0.38-pz6))); // (X, Y, Z)
    Wjoint_LED_[5].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.07, -0.14, 0.38-pz6))); // (X, Y, Z)
    Wjoint_LED_[6].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07, -0.14, 0.38-pz6))); // (X, Y, Z)


    // LEFT 
    // Body Front
    Wjoint_LED_[7].SetParentLink(&m_Link[HumeID::SIM_HIP]);
    Wjoint_LED_[7].SetChildLink(&LED_[7]);
    Wjoint_LED_[7].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07, 0.0, -0.02- pz6)));
    // Thigh Up
    Wjoint_LED_[8].SetParentLink(&m_Link[HumeID::SIM_LEFT_THIGH]);
    Wjoint_LED_[8].SetChildLink(&LED_[8]);
    Wjoint_LED_[8].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07, 0.0, 0.075)));
    // Thigh Below
    Wjoint_LED_[9].SetParentLink(&m_Link[HumeID::SIM_LEFT_THIGH]);
    Wjoint_LED_[9].SetChildLink(&LED_[9]);
    Wjoint_LED_[9].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07-px8, 0.0 - py8, -0.4-pz8)));
    // Shank Behind
    Wjoint_LED_[10].SetParentLink(&m_Link[HumeID::SIM_LEFT_CALF]);
    Wjoint_LED_[10].SetChildLink(&LED_[10]);
    Wjoint_LED_[10].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.013-px9, 0.0, -0.05-pz9)));
    // LEFT Foot
    Wjoint_LED_[11].SetParentLink(&m_Link[HumeID::SIM_LEFT_CALF]);
    Wjoint_LED_[11].SetChildLink(&LED_[11]);
    Wjoint_LED_[11].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.02-px9, 0.0, -0.45-pz9)));

    // Right
    // Body Back
    Wjoint_LED_[12].SetParentLink(&m_Link[HumeID::SIM_HIP]);
    Wjoint_LED_[12].SetChildLink(&LED_[12]);
    Wjoint_LED_[12].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.07, 0.0, -0.02 -pz6)));
    // Thigh Up
    Wjoint_LED_[13].SetParentLink(&m_Link[HumeID::SIM_RIGHT_THIGH]);
    Wjoint_LED_[13].SetChildLink(&LED_[13]);
    Wjoint_LED_[13].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07, 0.0, 0.075)));
    // Thigh Below
    Wjoint_LED_[14].SetParentLink(&m_Link[HumeID::SIM_RIGHT_THIGH]);
    Wjoint_LED_[14].SetChildLink(&LED_[14]);
    Wjoint_LED_[14].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(0.07-px8, 0.0-py8, -0.4-pz8)));
    // Shank Behind
    Wjoint_LED_[15].SetParentLink(&m_Link[HumeID::SIM_RIGHT_CALF]);
    Wjoint_LED_[15].SetChildLink(&LED_[15]);
    Wjoint_LED_[15].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.013-px9, 0.0, -0.05-pz9)));
    // Right Foot
    Wjoint_LED_[16].SetParentLink(&m_Link[HumeID::SIM_RIGHT_CALF]);
    Wjoint_LED_[16].SetChildLink(&LED_[16]);
    Wjoint_LED_[16].SetParentLinkFrame(EulerZYX(Vec3(), Vec3(-0.02-px9, -0.0, -0.45-pz9)));

}
void Hume::_SetInitialConf()
{
    double hip_angle(30.0);
    double knee_angle(70.0);
    
    // /////    General Pos 1 ///////
    switch (sim_setting->initial_pos_) {
    case 0:
        starting_height_ = 0.965;
        m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_State.m_rValue[0] = DEG2RAD(5.0);
        m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_State.m_rValue[0] = DEG2RAD(-5.0);
        m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_State.m_rValue[0] = DEG2RAD(30.0);

        m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_State.m_rValue[0] = DEG2RAD(-7.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_State.m_rValue[0] = DEG2RAD(-15.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_State.m_rValue[0] = DEG2RAD(25.0);

        m_Pjoint[HumeID::SIM_X].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Y].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Z].m_State.m_rValue[0] = BOOM_height*0.5;

        m_Rjoint[HumeID::SIM_Rx].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Ry].m_State.m_rValue[0] = 0.0; //HUME_TILTING;
        m_Rjoint[HumeID::SIM_Rz].m_State.m_rValue[0] = 0.0;
        break;

    case 1:
        starting_height_ = 1.2  -0.35419 + SIM_FOOT_RADIUS +0.05;
        hume_tilting_ = 0.0;
        m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_State.m_rValue[0] = DEG2RAD(2.0);
        m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_State.m_rValue[0] = DEG2RAD(-hip_angle-25);
        m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_State.m_rValue[0] = DEG2RAD(knee_angle);

        m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_State.m_rValue[0] = DEG2RAD(-2.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_State.m_rValue[0] = DEG2RAD(-hip_angle + 10.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_State.m_rValue[0] = DEG2RAD(knee_angle + 0.0);

        m_Pjoint[HumeID::SIM_X].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Y].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Z].m_State.m_rValue[0] = BOOM_height*0.5;// 1.0  -0.3617 + SIM_FOOT_RADIUS;

        m_Rjoint[HumeID::SIM_Rx].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Ry].m_State.m_rValue[0] = hume_tilting_; //HUME_TILTING;
        m_Rjoint[HumeID::SIM_Rz].m_State.m_rValue[0] = DEG2RAD(0.0);
        break;

    case 2:
        starting_height_ = 1.0; //  -0.35419 + SIM_FOOT_RADIUS;
        hume_tilting_ = 0.1;
        m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_State.m_rValue[0] = DEG2RAD(2.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_State.m_rValue[0] = DEG2RAD(-hip_angle);
        m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_State.m_rValue[0] = DEG2RAD(knee_angle);

        m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_State.m_rValue[0] = DEG2RAD(-2.0);
        m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_State.m_rValue[0] = DEG2RAD(-hip_angle );
        m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_State.m_rValue[0] = DEG2RAD(knee_angle + 0.0);

        m_Pjoint[HumeID::SIM_X].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Y].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Z].m_State.m_rValue[0] = BOOM_height*0.5;// 1.0  -0.3617 + SIM_FOOT_RADIUS;

        m_Rjoint[HumeID::SIM_Rx].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Ry].m_State.m_rValue[0] = hume_tilting_; //HUME_TILTING;
        m_Rjoint[HumeID::SIM_Rz].m_State.m_rValue[0] = 0.0;
        break;
// Actual robot Start Pose        
    case 3:
        starting_height_ = 1.0 - 0.419 + SIM_FOOT_RADIUS;
        hume_tilting_ = DEG2RAD(-5.0);
        m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_State.m_rValue[0] = 0.03;
        m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_State.m_rValue[0] = -0.71;
        m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_State.m_rValue[0] = 1.73;

        m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_State.m_rValue[0] = -0.02;
        m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_State.m_rValue[0] = -0.71;
        m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_State.m_rValue[0] = 1.73;

        m_Pjoint[HumeID::SIM_X].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Y].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Z].m_State.m_rValue[0] = BOOM_height*0.5;

        m_Rjoint[HumeID::SIM_Rx].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Ry].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Rz].m_State.m_rValue[0] = 0.0;
        break;
// Standing Pose
    case 4:
        starting_height_ = 1.0 - 0.219 + SIM_FOOT_RADIUS;
        hume_tilting_ = DEG2RAD(-5.0);
        m_Rjoint[HumeID::SIM_J_LEFT_HIP1].m_State.m_rValue[0] = DEG2RAD(0.0);
        m_Rjoint[HumeID::SIM_J_LEFT_HIP2].m_State.m_rValue[0] = DEG2RAD(-25.0);
        m_Rjoint[HumeID::SIM_J_LEFT_KNEE].m_State.m_rValue[0] = DEG2RAD(55.0);

        m_Rjoint[HumeID::SIM_J_RIGHT_HIP1].m_State.m_rValue[0] = DEG2RAD(-0.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_HIP2].m_State.m_rValue[0] = DEG2RAD( -25.0);
        m_Rjoint[HumeID::SIM_J_RIGHT_KNEE].m_State.m_rValue[0] = DEG2RAD( 55.0);

        m_Pjoint[HumeID::SIM_X].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Y].m_State.m_rValue[0] = 0.0;
        m_Pjoint[HumeID::SIM_Z].m_State.m_rValue[0] = BOOM_height*0.5;

        m_Rjoint[HumeID::SIM_Rx].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Ry].m_State.m_rValue[0] = 0.0;
        m_Rjoint[HumeID::SIM_Rz].m_State.m_rValue[0] = 0.0;
        break;

    }
    
    KIN_UpdateFrame_All_The_Entity();
}


void Hume::_SetInertia(){
    Inertia dummy = Inertia(0.0);
    m_Link[HumeID::SIM_Base].SetInertia(dummy);
    m_Link[HumeID::SIM_L_Y].SetInertia(dummy);
    m_Link[HumeID::SIM_L_Z].SetInertia(dummy);
    m_Link[HumeID::SIM_L_Rz].SetInertia(dummy);
    m_Link[HumeID::SIM_L_Ry].SetInertia(dummy);
    m_Link[HumeID::SIM_L_Rx].SetInertia(dummy);

    m_Link[HumeID::SIM_LFOOT].SetInertia(dummy);
    m_Link[HumeID::SIM_RFOOT].SetInertia(dummy);

    for (int i(0); i<SIM_NUM_LED; ++i){
        LED_[i].SetInertia(dummy);
    }
    
    Inertia inertia_body = Inertia (M_body, 0.3006166667, 0.2383713333,  0.09690466667); //Without Boom
    // xx, yy, zz, xy, yz, zx
    Inertia inertia_abduction = Inertia( 0.003734834, 0.013912108,   0.013551856,
                                        -0.00009107,  0.0000196188, -0.000302016);
    inertia_abduction.SetMass(M_abduction);
    //////////
    Inertia inertia_thigh  = Inertia( 38216989.0/1000000000.0, 1593401.0/40000000.0, 182353/40000000.0,  -10107.0/500000000.0, 57353.0/1000000000.0, -750079.0/250000000.0);
    inertia_thigh.SetMass(M_Thigh);

    //////////
    Inertia inertia_calf = Inertia(M_calf, 0.00338, 0.00338, 0.000011);

    //////////
    m_Link[HumeID::SIM_HIP].SetInertia(inertia_body);
    //
    m_Link[HumeID::SIM_RIGHT_HIP_SIDE].SetInertia(inertia_abduction);
    m_Link[HumeID::SIM_RIGHT_THIGH].SetInertia(inertia_thigh);
    m_Link[HumeID::SIM_RIGHT_CALF].SetInertia(inertia_calf);
    //             
    m_Link[HumeID::SIM_LEFT_HIP_SIDE].SetInertia(inertia_abduction);
    m_Link[HumeID::SIM_LEFT_THIGH].SetInertia(inertia_thigh);
    m_Link[HumeID::SIM_LEFT_CALF].SetInertia(inertia_calf);
}

void Hume::_SetLinkShape(){
    double body_offset(0.0);
/////////////////////////////    
    m_Link[HumeID::SIM_HIP].GetGeomInfo().SetShape(srGeometryInfo::TDS);
    m_Link[HumeID::SIM_HIP].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/body.3ds");
    m_Link[HumeID::SIM_HIP].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, SR_PI, 0.0), Vec3(-0.091,-0.2137, -0.27 -body_offset)));
//////////////////////////////
    m_Link[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().SetShape(srGeometryInfo::TDS);
    m_Link[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/abduction.3ds");
    m_Link[HumeID::SIM_RIGHT_HIP_SIDE].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF ,0.0, SR_PI), Vec3( -0.135, -0.085, -0.086 + body_offset)));

    m_Link[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().SetShape(srGeometryInfo::TDS);
    m_Link[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/thigh.3ds");
    m_Link[HumeID::SIM_RIGHT_THIGH].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(-0.1, -0.055,-0.312 + body_offset)));

    m_Link[HumeID::SIM_RIGHT_CALF].GetGeomInfo().SetShape(srGeometryInfo::TDS);
    m_Link[HumeID::SIM_RIGHT_CALF].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/knee_middle.3ds");
    m_Link[HumeID::SIM_RIGHT_CALF].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(-0.035, -0.038, -0.2135)));
//////////////////////////////////////    
   m_Link[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().SetShape(srGeometryInfo::TDS);
   m_Link[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/abduction.3ds");
   m_Link[HumeID::SIM_LEFT_HIP_SIDE].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF ,0.0,  SR_PI), Vec3( -0.135, -0.085, -0.086 + body_offset)));

   m_Link[HumeID::SIM_LEFT_THIGH].GetGeomInfo().SetShape(srGeometryInfo::TDS);
   m_Link[HumeID::SIM_LEFT_THIGH].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/thigh.3ds");
   m_Link[HumeID::SIM_LEFT_THIGH].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3( -0.1, -0.055, -0.312 + body_offset)));

   m_Link[HumeID::SIM_LEFT_CALF].GetGeomInfo().SetShape(srGeometryInfo::TDS);
   m_Link[HumeID::SIM_LEFT_CALF].GetGeomInfo().SetFileName(THIS_COM"src/ControlSystem/Hume_Controller/Hume_CAD/knee_middle.3ds");
   m_Link[HumeID::SIM_LEFT_CALF].GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(-0.035, -0.038, -0.2135)));


}


void Hume::_SetPassiveJoint(srJoint::ACTTYPE joint_type){

    float Link_R = (rand()%100)*0.008f;
    float Link_G = (rand()%100)*0.011f;
    float Link_B = (rand()%100)*0.012f; 

    double passive_radius(0.001);
    double passive_length(0.001);
    //Passive Joint (PRISMATIC)
    m_Pjoint[HumeID::SIM_X].SetParentLink(&m_Link[HumeID::SIM_Base]);
    m_Pjoint[HumeID::SIM_X].SetChildLink( &m_Link[HumeID::SIM_L_Y]);
    m_Pjoint[HumeID::SIM_X].GetGeomInfo().SetColor(Link_R, Link_G, 0.0);
    m_Pjoint[HumeID::SIM_X].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Pjoint[HumeID::SIM_X].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);

    m_Pjoint[HumeID::SIM_X].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_Pjoint[HumeID::SIM_X].SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    m_Pjoint[HumeID::SIM_Y].SetParentLink(&m_Link[HumeID::SIM_L_Y]);
    m_Pjoint[HumeID::SIM_Y].SetChildLink( &m_Link[HumeID::SIM_L_Z]);
    m_Pjoint[HumeID::SIM_Y].GetGeomInfo().SetColor(0.0, Link_G, Link_B);
    m_Pjoint[HumeID::SIM_Y].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Pjoint[HumeID::SIM_Y].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
        
    m_Pjoint[HumeID::SIM_Y].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_Pjoint[HumeID::SIM_Y].SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    m_Pjoint[HumeID::SIM_Z].SetParentLink(&m_Link[HumeID::SIM_L_Z]);
    m_Pjoint[HumeID::SIM_Z].SetChildLink( &m_Link[HumeID::SIM_L_Rz]);
    m_Pjoint[HumeID::SIM_Z].GetGeomInfo().SetColor(Link_R, 0.0, Link_B);
    m_Pjoint[HumeID::SIM_Z].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_Pjoint[HumeID::SIM_Z].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
                
    m_Pjoint[HumeID::SIM_Z].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_Pjoint[HumeID::SIM_Z].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));


    //Passive Joint (ROTATION)
    m_Rjoint[HumeID::SIM_Rz].SetParentLink(&m_Link[HumeID::SIM_L_Rz]);
    m_Rjoint[HumeID::SIM_Rz].SetChildLink( &m_Link[HumeID::SIM_L_Ry]);
    m_Rjoint[HumeID::SIM_Rz].GetGeomInfo().SetColor(Link_R, 0.0, 0.0);
    m_Rjoint[HumeID::SIM_Rz].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_Rjoint[HumeID::SIM_Rz].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    // m_Rjoint[HumeID::SIM_Rz].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_Rjoint[HumeID::SIM_Rz].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),
                                                         Vec3(0.0, 0.0, starting_height_)));
    m_Rjoint[HumeID::SIM_Rz].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));


    m_Rjoint[HumeID::SIM_Ry].SetParentLink(&m_Link[HumeID::SIM_L_Ry]);
    m_Rjoint[HumeID::SIM_Ry].SetChildLink( &m_Link[HumeID::SIM_L_Rx]);
    m_Rjoint[HumeID::SIM_Ry].GetGeomInfo().SetColor(0.0, Link_G, 0.0);
    m_Rjoint[HumeID::SIM_Ry].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_Rjoint[HumeID::SIM_Ry].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    m_Rjoint[HumeID::SIM_Ry].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_Rjoint[HumeID::SIM_Ry].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));

    // m_Rjoint[HumeID::SIM_Ry].m_PosLimit[1] = RAD2DEG(0.12);
    
    m_Rjoint[HumeID::SIM_Rx].SetParentLink(&m_Link[HumeID::SIM_L_Rx]);
    m_Rjoint[HumeID::SIM_Rx].SetChildLink( &m_Link[HumeID::SIM_HIP]);
    m_Rjoint[HumeID::SIM_Rx].GetGeomInfo().SetColor(0.0, 0.0, Link_B);
    m_Rjoint[HumeID::SIM_Rx].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_Rjoint[HumeID::SIM_Rx].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    m_Rjoint[HumeID::SIM_Rx].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_Rjoint[HumeID::SIM_Rx].SetChildLinkFrame(EulerZYX(Vec3(0.0, -SR_PI_HALF, SR_PI), Vec3(0.0, 0.0, -pz6 ) ) );
    
    for (int i(0); i<SIM_NUM_PASSIVE; ++i){
        m_Link[i + HumeID::SIM_Base].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        m_Link[i + HumeID::SIM_Base].GetGeomInfo().SetDimension(0.001, 0.001, 0);
    }

    // Add Collisiion
    if(sim_setting->hanging_){    
    // Boom Slider
    // m_Link[HumeID::SIM_L_Ry].GetGeomInfo().SetShape(srGeometryInfo::BOX);
    // m_Link[HumeID::SIM_L_Ry].GetGeomInfo().SetDimension(0.063, 0.8, 0.384);
    // m_Rjoint[HumeID::SIM_Rz].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(-0.1, 0.0, 0.0)));
    // m_Rjoint[HumeID::SIM_Ry].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    //
    // Inertia slider = Inertia(5.786, 0.368, 0.0, 0.368); // Only for Mass, No meaning..Inertia
    // m_Link[HumeID::SIM_L_Y].SetInertia(slider);
    // Inertia linkage = Inertia(2.69, 0.45, 0.25, 0.7); // Only for Mass, No meaning...Inertia
    // m_Link[HumeID::SIM_L_Ry].SetInertia(linkage);
    //
    m_Link[HumeID::SIM_L_Rz].GetGeomInfo().SetDimension(0.05, BOOM_height, 0.0);
    m_Collision[HumeID::SIM_L_Rz].GetGeomInfo().SetShape(m_Link[HumeID::SIM_L_Rz].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_L_Rz].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_L_Rz].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_L_Rz].AddCollision(& m_Collision[HumeID::SIM_L_Rz]);
    }

    ////////////// Actuation Type (Torque.. Default) ////////
    m_Pjoint[HumeID::SIM_X].SetActType(joint_type);
    m_Pjoint[HumeID::SIM_Y].SetActType(joint_type);
    m_Pjoint[HumeID::SIM_Z].SetActType(joint_type);

    m_Rjoint[HumeID::SIM_Rx].SetActType(joint_type);
    m_Rjoint[HumeID::SIM_Ry].SetActType(joint_type);
    m_Rjoint[HumeID::SIM_Rz].SetActType(joint_type);

}

void Hume::_Foot_Assemble(){
    //Foot Weld Joint
    m_Wjoint[HumeID::WJ_LFOOT].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
    m_Wjoint[HumeID::WJ_LFOOT].GetGeomInfo().SetDimension(0.01,0.01, 0.0);
    m_Wjoint[HumeID::WJ_LFOOT].SetParentLink(&m_Link[HumeID::SIM_LEFT_CALF]);
    m_Wjoint[HumeID::WJ_LFOOT].SetChildLink (&m_Link[HumeID::SIM_LFOOT]);
    m_Wjoint[HumeID::WJ_LFOOT].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -(m_leng_calf + pz9))));
    m_Wjoint[HumeID::WJ_LFOOT].SetChildLinkFrame(EulerZYX(Vec3(0.0,0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));
                                                      
        
    //Foot
    m_Link[HumeID::SIM_LFOOT].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
    m_Link[HumeID::SIM_LFOOT].GetGeomInfo().SetDimension(SIM_FOOT_RADIUS*2, SIM_FOOT_RADIUS*1.3 , 0.0);
//	m_Link[HumeID::LFOOT].GetGeomInfo().SetColor(Link_R, Link_G, Link_B);

    m_Link[HumeID::SIM_RFOOT].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
    m_Link[HumeID::SIM_RFOOT].GetGeomInfo().SetDimension(SIM_FOOT_RADIUS*2, SIM_FOOT_RADIUS*1.3 , 0.0);
//	m_Link[HumeID::RFOOT].GetGeomInfo().SetColor(Link_R, Link_G, Link_B);

    double restitution(0.0);
    double friction(20.0);
    double damping(0.7);
    m_Link[HumeID::SIM_LFOOT].SetFriction(friction);
    m_Link[HumeID::SIM_LFOOT].SetDamping(damping);
    m_Link[HumeID::SIM_LFOOT].SetRestitution(restitution);

    m_Link[HumeID::SIM_RFOOT].SetFriction(friction);
    m_Link[HumeID::SIM_RFOOT].SetDamping(damping);
    m_Link[HumeID::SIM_RFOOT].SetRestitution(restitution);

//Foot Weld Joint
    m_Wjoint[HumeID::WJ_RFOOT].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
    m_Wjoint[HumeID::WJ_RFOOT].GetGeomInfo().SetDimension(0.02, 0.0, 0.0);
    m_Wjoint[HumeID::WJ_RFOOT].SetParentLink(&m_Link[HumeID::SIM_RIGHT_CALF]);
    m_Wjoint[HumeID::WJ_RFOOT].SetChildLink(&m_Link[HumeID::SIM_RFOOT]);
    m_Wjoint[HumeID::WJ_RFOOT].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -(m_leng_calf + pz9))));
    m_Wjoint[HumeID::WJ_RFOOT].SetChildLinkFrame(EulerZYX(Vec3(0.0,0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));


    ////// Collision  ///////////
    //Left Foot
    m_Collision[HumeID::SIM_LFOOT].GetGeomInfo().SetShape(m_Link[HumeID::SIM_LFOOT].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_LFOOT].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_LFOOT].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_LFOOT].AddCollision(&m_Collision[HumeID::SIM_LFOOT]);
    
    //Right Foot
    m_Collision[HumeID::SIM_RFOOT].GetGeomInfo().SetShape(m_Link[HumeID::SIM_RFOOT].GetGeomInfo().GetShape());
    m_Collision[HumeID::SIM_RFOOT].GetGeomInfo().SetDimension(m_Link[HumeID::SIM_RFOOT].GetGeomInfo().GetDimension());
    m_Link[HumeID::SIM_RFOOT].AddCollision(&m_Collision[HumeID::SIM_RFOOT]);
}
