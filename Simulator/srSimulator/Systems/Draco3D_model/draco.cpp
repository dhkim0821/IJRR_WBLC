#include "draco.h"

srDraco::srDraco():
  SystemGenerator()
{
    printf("[DRACO] ASSEMBLED\n");
}

srDraco::~srDraco(){
}

void srDraco::_SetJointLimit(){
}

void srDraco::_SetCollision(){
    collision_.resize(1);
    for (int i = 0; i < collision_.size(); ++i) {
        collision_[i] = new srCollision();
        collision_[i]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
      collision_[i]->GetGeomInfo().SetDimension(0.25, 0.07, 0.02);
    }

    link_[link_idx_map_.find("foot")->second]->AddCollision(collision_[0]);

    double fric(0.8);
    link_[link_idx_map_.find("foot")->second]->SetFriction(fric);

    double damp(0.01);
    link_[link_idx_map_.find("foot")->second]->SetDamping(damp);

    double restit(0.0);
    link_[link_idx_map_.find("foot")->second]->SetRestitution(restit);
}

void srDraco::_SetInitialConf(){
    vp_joint_[2]->m_State.m_rValue[0] = 0.67;
    r_joint_[r_joint_idx_map_.find("bodyPitch")->second]->m_State.m_rValue[0] = -1;
    r_joint_[r_joint_idx_map_.find("kneePitch")->second]->m_State.m_rValue[0] = 2;
    r_joint_[r_joint_idx_map_.find("ankle")->second]->m_State.m_rValue[0] = -1;
    KIN_UpdateFrame_All_The_Entity();
}
