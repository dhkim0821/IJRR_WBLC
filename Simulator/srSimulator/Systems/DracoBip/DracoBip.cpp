#include "DracoBip.h"
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

DracoBip::DracoBip():SystemGenerator(), 
    //hanging_height_( 1.084217  + 0.3 )
    hanging_height_( 1.084217 ),
    collision_offset_(0.6),
    initial_posture_(0)
{
  ParamHandler handler(DracoBipConfigPath"SIM_sr_sim_setting.yaml");
  handler.getInteger("initial_posture", initial_posture_);
  handler.getValue("hanging_height", hanging_height_);

  printf("[DracoBip] ASSEMBLED\n");
}

DracoBip::~DracoBip(){
}

void DracoBip::_SetJointLimit(){
}

void DracoBip::_SetCollision(){
  collision_.resize(4 + 1);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  //collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
  collision_[0]->GetGeomInfo().SetDimension(0.055, 0.03, 0.0);
  collision_[0]->SetLocalFrame(EulerZYX(Vec3(0,0,M_PI/2.0), Vec3(0.07, 0, 0)));
 
  //collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
  collision_[1]->GetGeomInfo().SetDimension(0.055, 0.03, 0.0);
  collision_[1]->SetLocalFrame(EulerZYX(Vec3(0,0,M_PI/2.0), Vec3(-0.06, 0, 0)));
  
  //collision_[2]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[2]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
  collision_[2]->GetGeomInfo().SetDimension(0.055, 0.03, 0.0);
  collision_[2]->SetLocalFrame(EulerZYX(Vec3(0,0,M_PI/2.0), Vec3(0.07, 0.0, 0)));

  //collision_[3]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[3]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
  collision_[3]->GetGeomInfo().SetDimension(0.055, 0.03, 0.0);
  collision_[3]->SetLocalFrame(EulerZYX(Vec3(0,0,M_PI/2.), Vec3(-0.06, 0., 0)));


  link_[link_idx_map_.find("rAnkle")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("rAnkle")->second]->AddCollision(collision_[1]);
  link_[link_idx_map_.find("lAnkle")->second]->AddCollision(collision_[2]);
  link_[link_idx_map_.find("lAnkle")->second]->AddCollision(collision_[3]);

  double fric(10.8);
  link_[link_idx_map_.find("rAnkle")->second]->SetFriction(fric);
  link_[link_idx_map_.find("lAnkle")->second]->SetFriction(fric);

  double damp(0.11);
  link_[link_idx_map_.find("rAnkle")->second]->SetDamping(damp);
  link_[link_idx_map_.find("lAnkle")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("rAnkle")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("lAnkle")->second]->SetRestitution(restit);

  // Link
  collision_[4] = new srCollision();
  collision_[4]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  collision_[4]->GetGeomInfo().SetDimension(0.05, 0.1, 0.05);
  collision_[4]->SetLocalFrame(
          EulerZYX(Vec3(0., -SR_PI_HALF, SR_PI), 
                   //Vec3( 0., 0. , -0.050 + hanging_height_  ) ) );
                   Vec3( -hanging_height_ + 0.05, 0. , 0.  ) ) );
  v_link_[5]->AddCollision(collision_[4]);
  v_link_[5]->SetFriction(10.);
  //link_[link_idx_map_.find("torso")->second]->AddCollision(collision_[4]);
  //link_[link_idx_map_.find("torso")->second]->SetFriction(fric);
}

void DracoBip::_SetInitialConf(){

    for(int i(0); i<3; ++i)    vr_joint_[i]->m_State.m_rValue[1] = 0.;
    for(int i(0); i<3; ++i)    vp_joint_[i]->m_State.m_rValue[1] = 0.;
    for(int i(0); i<num_act_joint_; ++i)    r_joint_[i]->m_State.m_rValue[1] = 0.;

  vp_joint_[0]->m_State.m_rValue[0] = 0.0; // X
  vp_joint_[1]->m_State.m_rValue[0] = 0.0; // Y

  // 0: Stand up posture
  //int initial_config_type(1);

  switch(initial_posture_){
    case 0:
        vp_joint_[2]->m_State.m_rValue[0] = 1.084217;
        vr_joint_[0]->m_State.m_rValue[0] = 0.0;
        vr_joint_[1]->m_State.m_rValue[0] = 0.0348;
        vr_joint_[2]->m_State.m_rValue[0] = 0.0;

        r_joint_[r_joint_idx_map_.find("lHipPitch")->second]->m_State.m_rValue[0] = -0.59;
        r_joint_[r_joint_idx_map_.find("lKnee")->second]->m_State.m_rValue[0] = 1.1;
        r_joint_[r_joint_idx_map_.find("lAnkle")->second]->m_State.m_rValue[0] = 1.03;

        r_joint_[r_joint_idx_map_.find("rHipPitch")->second]->m_State.m_rValue[0] = -0.59;
        r_joint_[r_joint_idx_map_.find("rKnee")->second]->m_State.m_rValue[0] = 1.1;
        r_joint_[r_joint_idx_map_.find("rAnkle")->second]->m_State.m_rValue[0] = 1.03;
        break;
    case 1:
        vp_joint_[2]->m_State.m_rValue[0] = hanging_height_;

        r_joint_[r_joint_idx_map_.find("lAnkle")->second]->
            m_State.m_rValue[0] = M_PI/2.;
        r_joint_[r_joint_idx_map_.find("rAnkle")->second]->
            m_State.m_rValue[0] = M_PI/2.;
         break;
    case 2:
         vp_joint_[2]->m_State.m_rValue[0] = hanging_height_;

        //r_joint_[r_joint_idx_map_.find("lHipRoll")->second]->m_State.m_rValue[0] = -0.5;
        r_joint_[r_joint_idx_map_.find("lHipPitch")->second]->
            m_State.m_rValue[0] = -0.5;
        r_joint_[r_joint_idx_map_.find("lKnee")->second]->
            m_State.m_rValue[0] = 1.8;
        r_joint_[r_joint_idx_map_.find("lAnkle")->second]->m_State.m_rValue[0] = 1.03;

        //r_joint_[r_joint_idx_map_.find("rHipRoll")->second]->m_State.m_rValue[0] = -0.5;
        r_joint_[r_joint_idx_map_.find("rHipPitch")->second]->
            m_State.m_rValue[0] = -0.5;
        r_joint_[r_joint_idx_map_.find("rKnee")->second]->
            m_State.m_rValue[0] = 1.8;
        r_joint_[r_joint_idx_map_.find("rAnkle")->second]->m_State.m_rValue[0] = 1.03;

        //r_joint_[r_joint_idx_map_.find("lAnkle")->second]->
            //m_State.m_rValue[0] = M_PI/2.;
        //r_joint_[r_joint_idx_map_.find("rAnkle")->second]->
            //m_State.m_rValue[0] = M_PI/2.;
         break;
  }
  KIN_UpdateFrame_All_The_Entity();
}
