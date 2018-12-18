#include "nao.h"

srNao::srNao():
  SystemGenerator()
{
  printf("[NAO] ASSEMBLED\n");
}

srNao::~srNao(){
}

void srNao::_SetJointLimit(){
}

void srNao::_SetCollision(){
    collision_.resize(2);
    for (int i = 0; i < collision_.size(); ++i) {
        collision_[i] = new srCollision();
    }
    collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    collision_[0]->GetGeomInfo().SetDimension(0.12, 0.06, 0.005);
    collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    collision_[1]->GetGeomInfo().SetDimension(0.12, 0.06, 0.005);

    link_[link_idx_map_.find("r_sole")->second]->AddCollision(collision_[0]);
    link_[link_idx_map_.find("l_sole")->second]->AddCollision(collision_[1]);

    double fric(0.8);
    link_[link_idx_map_.find("r_sole")->second]->SetFriction(fric);
    link_[link_idx_map_.find("l_sole")->second]->SetFriction(fric);

    double damp(0.05);
    link_[link_idx_map_.find("r_sole")->second]->SetDamping(damp);
    link_[link_idx_map_.find("l_sole")->second]->SetDamping(damp);

    double restit(0.0);
    link_[link_idx_map_.find("r_sole")->second]->SetRestitution(restit);
    link_[link_idx_map_.find("l_sole")->second]->SetRestitution(restit);

}

void srNao::_SetInitialConf(){
    vp_joint_[0]->m_State.m_rValue[0] = 0.0;
    vp_joint_[1]->m_State.m_rValue[0] = 0.0;
    vp_joint_[2]->m_State.m_rValue[0] = 0.376;
    vr_joint_[0]->m_State.m_rValue[0] = 0.0;
    vr_joint_[1]->m_State.m_rValue[0] = 0.0;
    vr_joint_[2]->m_State.m_rValue[0] = 0.0;

    //case 0 : just stand
    int pose(0);

    switch (pose) {
        case 0:
            vp_joint_[2]->m_State.m_rValue[0] = 0.376-0.004;
            r_joint_[r_joint_idx_map_.find("LShoulderPitch")->second]->m_State.m_rValue[0] = 1.5;
            r_joint_[r_joint_idx_map_.find("RShoulderPitch")->second]->m_State.m_rValue[0] = 1.5;
            r_joint_[r_joint_idx_map_.find("LShoulderRoll")->second]->m_State.m_rValue[0] = 0.5;
            r_joint_[r_joint_idx_map_.find("RShoulderRoll")->second]->m_State.m_rValue[0] = -0.5;
            r_joint_[r_joint_idx_map_.find("LElbowYaw")->second]->m_State.m_rValue[0] = -1.5;
            r_joint_[r_joint_idx_map_.find("RElbowYaw")->second]->m_State.m_rValue[0] = 1.5;
            r_joint_[r_joint_idx_map_.find("LKneePitch")->second]->m_State.m_rValue[0] = 0.3;
            r_joint_[r_joint_idx_map_.find("RKneePitch")->second]->m_State.m_rValue[0] = 0.3;
            r_joint_[r_joint_idx_map_.find("LAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
            r_joint_[r_joint_idx_map_.find("RAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
            break;
    }

    KIN_UpdateFrame_All_The_Entity();
}
