#include "SagitP3.hpp"
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

SagitP3::SagitP3():SystemGenerator(), 
    //hanging_height_( 1.084217  + 0.3 )
    hanging_height_( 1.084217 ),
    collision_offset_(0.6),
    initial_posture_(0)
{
  ParamHandler handler(SagitP3ConfigPath"SIM_sr_sim_setting.yaml");
  handler.getInteger("initial_posture", initial_posture_);
  handler.getValue("hanging_height", hanging_height_);
}

SagitP3::~SagitP3(){
}

void SagitP3::_SetJointLimit(){
}

void SagitP3::_SetCollision(){
  collision_.resize(2 + 1);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[0]->GetGeomInfo().SetDimension(0.25, 0.02, 0.25);
  collision_[0]->SetLocalFrame(EulerZYX(Vec3(0,0,0), Vec3(-0.0, -0.03, 0)));
 
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[1]->GetGeomInfo().SetDimension(0.25, 0.02, 0.25);
  collision_[1]->SetLocalFrame(EulerZYX(Vec3(0,0,0), Vec3(-0.0, -0.03, 0)));
  
  link_[link_idx_map_.find("right_foot_link")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("left_foot_link")->second]->AddCollision(collision_[1]);

  double fric(0.8);
  link_[link_idx_map_.find("right_foot_link")->second]->SetFriction(fric);
  link_[link_idx_map_.find("left_foot_link")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("right_foot_link")->second]->SetDamping(damp);
  link_[link_idx_map_.find("left_foot_link")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("right_foot_link")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("left_foot_link")->second]->SetRestitution(restit);

  // Link
  collision_[2] = new srCollision();
  collision_[2]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  collision_[2]->GetGeomInfo().SetDimension(0.05, 0.1, 0.05);
  collision_[2]->SetLocalFrame(
          EulerZYX(Vec3(0., -SR_PI_HALF, SR_PI), 
                   //Vec3( 0., 0. , -0.050 + hanging_height_  ) ) );
                   Vec3( -hanging_height_ + 0.05, 0. , 0.  ) ) );
                   //Vec3( -hanging_height_+ 0.2, 0. , 0.  ) ) );
  v_link_[5]->AddCollision(collision_[2]);
  v_link_[5]->SetFriction(10.);
}

void SagitP3::_SetInitialConf(){

    for(int i(0); i<3; ++i)    vr_joint_[i]->m_State.m_rValue[1] = 0.;
    for(int i(0); i<3; ++i)    vp_joint_[i]->m_State.m_rValue[1] = 0.;
    for(int i(0); i<num_act_joint_; ++i)    r_joint_[i]->m_State.m_rValue[1] = 0.;

    vp_joint_[0]->m_State.m_rValue[0] = 0.0; // X
    vp_joint_[1]->m_State.m_rValue[0] = 0.0; // Y
    vp_joint_[2]->m_State.m_rValue[0] = hanging_height_;

    vr_joint_[0]->m_State.m_rValue[0] = 0.;// SR_PI_HALF; // Z
    vr_joint_[1]->m_State.m_rValue[0] = 0.; // Y
    vr_joint_[2]->m_State.m_rValue[0] = SR_PI_HALF;
    
//printf("left flexion idx: %d\n", r_joint_idx_map_.find("Sagit_P3_Flexion_Left")->second);
//printf("right flexion idx: %d\n", r_joint_idx_map_.find("Sagit_P3_Flexion_Right")->second);
    r_joint_[r_joint_idx_map_.find("Sagit_P3_Flexion_Left")->second]->
        m_State.m_rValue[0] = -0.31;
        //m_State.m_rValue[0] = 0.;
    r_joint_[r_joint_idx_map_.find("Sagit_P3_Flexion_Right")->second]->
        m_State.m_rValue[0] = -0.31;
        //m_State.m_rValue[0] = 0.;

    r_joint_[r_joint_idx_map_.find("Sagit_P3_Knee_Left")->second]->
        m_State.m_rValue[0] = 1.08;
    r_joint_[r_joint_idx_map_.find("Sagit_P3_Knee_Right")->second]->
        m_State.m_rValue[0] = 1.08;

    r_joint_[r_joint_idx_map_.find("Sagit_P3_Ankle_Left")->second]->
        m_State.m_rValue[0] = -0.78;
    r_joint_[r_joint_idx_map_.find("Sagit_P3_Ankle_Right")->second]->
        m_State.m_rValue[0] = -0.78;

    r_joint_[r_joint_idx_map_.find("Sagit_P3_Ankle_Roll_Left_Passive")->second]->
        m_State.m_rValue[0] = -0.092;
    r_joint_[r_joint_idx_map_.find("Sagit_P3_Ankle_Roll_Right_Passive")->second]->
        m_State.m_rValue[0] = 0.092;

    //switch(initial_posture_){
    //case 0:
    //vp_joint_[2]->m_State.m_rValue[0] = 1.084217;
    //vr_joint_[0]->m_State.m_rValue[0] = 0.0;
        //vr_joint_[1]->m_State.m_rValue[0] = 0.0348;
        //vr_joint_[2]->m_State.m_rValue[0] = 0.0;

        //r_joint_[r_joint_idx_map_.find("lHipPitch")->second]->m_State.m_rValue[0] = -0.59;
        //r_joint_[r_joint_idx_map_.find("lKnee")->second]->m_State.m_rValue[0] = 1.1;
        //r_joint_[r_joint_idx_map_.find("left_foot_link")->second]->m_State.m_rValue[0] = 1.03;

        //r_joint_[r_joint_idx_map_.find("rHipPitch")->second]->m_State.m_rValue[0] = -0.59;
        //r_joint_[r_joint_idx_map_.find("rKnee")->second]->m_State.m_rValue[0] = 1.1;
        //r_joint_[r_joint_idx_map_.find("right_foot_link")->second]->m_State.m_rValue[0] = 1.03;
        //break;
    //case 1:

        //r_joint_[r_joint_idx_map_.find("left_foot_link")->second]->
            //m_State.m_rValue[0] = M_PI/2.;
        //r_joint_[r_joint_idx_map_.find("right_foot_link")->second]->
            //m_State.m_rValue[0] = M_PI/2.;
         //break;
    //case 2:
         //vp_joint_[2]->m_State.m_rValue[0] = hanging_height_;

        //r_joint_[r_joint_idx_map_.find("lHipPitch")->second]->m_State.m_rValue[0] = -0.5;
        //r_joint_[r_joint_idx_map_.find("lKnee")->second]->m_State.m_rValue[0] = 1.4;
        //r_joint_[r_joint_idx_map_.find("left_foot_link")->second]->m_State.m_rValue[0] = 1.03;

        //r_joint_[r_joint_idx_map_.find("rHipPitch")->second]->m_State.m_rValue[0] = -0.5;
        //r_joint_[r_joint_idx_map_.find("rKnee")->second]->m_State.m_rValue[0] = 1.4;
        //r_joint_[r_joint_idx_map_.find("right_foot_link")->second]->m_State.m_rValue[0] = 1.03;

         //break;
  //}
  KIN_UpdateFrame_All_The_Entity();
}
