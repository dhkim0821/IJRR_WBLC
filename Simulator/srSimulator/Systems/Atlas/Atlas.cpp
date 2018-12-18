#include "Atlas.h"

Atlas::Atlas():SystemGenerator()
{
  printf("[Atlas] ASSEMBLED\n");
}

Atlas::~Atlas(){
}

void Atlas::_SetJointLimit(){
}

void Atlas::_SetCollision(){
  collision_.resize(2);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[0]->GetGeomInfo().SetDimension(0.23, 0.16, 0.03);
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[1]->GetGeomInfo().SetDimension(0.23, 0.16, 0.03);

  link_[link_idx_map_.find("r_foot")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("l_foot")->second]->AddCollision(collision_[1]);

  double fric(0.8);
  link_[link_idx_map_.find("r_foot")->second]->SetFriction(fric);
  link_[link_idx_map_.find("l_foot")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("r_foot")->second]->SetDamping(damp);
  link_[link_idx_map_.find("l_foot")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("r_foot")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("l_foot")->second]->SetRestitution(restit);
}

void Atlas::_SetInitialConf(){
  //case 0 : just stand
  //case 1 : left leg up
  //case 2 : lower stand
  //case 3: walking ready
  //case 4 : ICRA 2018

  int pose(2);

  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  vp_joint_[2]->m_State.m_rValue[0] = 1.135;// + 0.3;
  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  r_joint_[r_joint_idx_map_.find("l_leg_hpy"  )->second]->m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("r_leg_hpy" )->second]->m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("l_leg_kny" )->second]->m_State.m_rValue[0] = 0.8;
  r_joint_[r_joint_idx_map_.find("r_leg_kny")->second]->m_State.m_rValue[0] = 0.8;
  r_joint_[r_joint_idx_map_.find("l_leg_aky")->second]->m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("r_leg_aky")->second]->m_State.m_rValue[0] = -0.4;

  r_joint_[r_joint_idx_map_.find("r_arm_shx")->second]->m_State.m_rValue[0] = 0.8;
  r_joint_[r_joint_idx_map_.find("r_arm_shy" )->second]->m_State.m_rValue[0] = -0.5;
  r_joint_[r_joint_idx_map_.find("r_arm_ely" )->second]->m_State.m_rValue[0] = -0.9;
  r_joint_[r_joint_idx_map_.find("r_arm_elx"   )->second]->m_State.m_rValue[0] = 1.8;

  r_joint_[r_joint_idx_map_.find("l_arm_shx" )->second]->m_State.m_rValue[0] = -0.8;
  r_joint_[r_joint_idx_map_.find("l_arm_shy" )->second]->m_State.m_rValue[0] = -0.5;
  r_joint_[r_joint_idx_map_.find("l_arm_ely"  )->second]->m_State.m_rValue[0] = -0.9;
  r_joint_[r_joint_idx_map_.find("l_arm_elx"    )->second]->m_State.m_rValue[0] = -1.8;

  switch(pose){
  case 0:

    break;

  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 1.131;
    break;

  case 2:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079 - 0.182 + 0.015 - 0.0271;
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
