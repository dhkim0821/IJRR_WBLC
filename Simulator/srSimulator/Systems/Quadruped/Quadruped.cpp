#include "Quadruped.h"

Quadruped::Quadruped():SystemGenerator()
{
  printf("[Quadruped] ASSEMBLED\n");
}

Quadruped::~Quadruped(){
}

void Quadruped::_SetJointLimit(){
}

void Quadruped::_SetCollision(){
  collision_.resize(4);
  for (int i = 0; i < collision_.size(); ++i) {
      collision_[i] = new srCollision();
      collision_[i]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
      collision_[i]->GetGeomInfo().SetDimension(0.05, 0., 0.);

  }

  link_[link_idx_map_.find("foot_fr")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("foot_fl")->second]->AddCollision(collision_[1]);
  link_[link_idx_map_.find("foot_hr")->second]->AddCollision(collision_[2]);
  link_[link_idx_map_.find("foot_hl")->second]->AddCollision(collision_[3]);

  double fric(0.8);
  link_[link_idx_map_.find("foot_fr")->second]->SetFriction(fric);
  link_[link_idx_map_.find("foot_fl")->second]->SetFriction(fric);
  link_[link_idx_map_.find("foot_hr")->second]->SetFriction(fric);
  link_[link_idx_map_.find("foot_hl")->second]->SetFriction(fric);


  double damp(0.01);
  link_[link_idx_map_.find("foot_fr")->second]->SetDamping(damp);
  link_[link_idx_map_.find("foot_fl")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("foot_fr")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("foot_fl")->second]->SetRestitution(restit);
}

void Quadruped::_SetInitialConf(){
  //case 0 : just stand
  //case 1 : left leg up
  //case 2 : lower stand
  //case 3: walking ready
  //case 4 : ICRA 2018

  int pose(2);

  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  vp_joint_[2]->m_State.m_rValue[0] = 0.5;
  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  for(int i(0); i< num_act_joint_; ++i) r_joint_[i]->m_State.m_rValue[0] = 0.;

  r_joint_[r_joint_idx_map_.find("thigh_fr_to_knee_fr_j")->second]->
      m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("thigh_fl_to_knee_fl_j")->second]->
      m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("thigh_hr_to_knee_hr_j")->second]->
      m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("thigh_hl_to_knee_hl_j")->second]->
      m_State.m_rValue[0] = -0.4;

  switch(pose){
  case 0:

    break;

  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 1.131;
    break;

  case 2:
    vp_joint_[2]->m_State.m_rValue[0] =  0.4;
    break;

  case 3:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.032720;
    vp_joint_[2]->m_State.m_rValue[0] =  1.050418;
    break;

  case 4:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079;

    break;
  }
  KIN_UpdateFrame_All_The_Entity();
}
