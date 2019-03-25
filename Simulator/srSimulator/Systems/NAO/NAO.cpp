#include "NAO.h"

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
    collision_[0]->GetGeomInfo().SetDimension(0.15, 0.07, 0.022);
    collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    collision_[1]->GetGeomInfo().SetDimension(0.15, 0.07, 0.022);

    link_[link_idx_map_.find("r_ankle")->second]->AddCollision(collision_[0]);
    link_[link_idx_map_.find("l_ankle")->second]->AddCollision(collision_[1]);

    double fric(0.8);
    link_[link_idx_map_.find("r_ankle")->second]->SetFriction(fric);
    link_[link_idx_map_.find("l_ankle")->second]->SetFriction(fric);

    double damp(0.05);
    link_[link_idx_map_.find("r_ankle")->second]->SetDamping(damp);
    link_[link_idx_map_.find("l_ankle")->second]->SetDamping(damp);

    double restit(0.0);
    link_[link_idx_map_.find("r_ankle")->second]->SetRestitution(restit);
    link_[link_idx_map_.find("l_ankle")->second]->SetRestitution(restit);

}

void srNao::_SetInitialConf(){
    vp_joint_[0]->m_State.m_rValue[0] = 0.0;
    vp_joint_[1]->m_State.m_rValue[0] = 0.0;
    vp_joint_[2]->m_State.m_rValue[0] = 0.376;
    vr_joint_[0]->m_State.m_rValue[0] = 0.0;
    vr_joint_[1]->m_State.m_rValue[0] = 0.0;
    vr_joint_[2]->m_State.m_rValue[0] = 0.0;
    std::map<std::string, int>::iterator iter = r_joint_idx_map_.begin();
    while(iter != r_joint_idx_map_.end()){
        r_joint_[iter->second]->m_State.m_rValue[0] = 0.;
        ++iter;
    }
    //case 0 : just stand
    int pose(0);
    //int pose(1);

    switch (pose) {
        case 0:
            vp_joint_[2]->m_State.m_rValue[0] = 0.326-0.004;
            r_joint_[r_joint_idx_map_.find("LShoulderPitch")->second]->
                m_State.m_rValue[0] = 1.5;
            r_joint_[r_joint_idx_map_.find("RShoulderPitch")->second]->
                m_State.m_rValue[0] = 1.5;
            r_joint_[r_joint_idx_map_.find("LShoulderRoll")->second]->
                m_State.m_rValue[0] = 0.5;
            r_joint_[r_joint_idx_map_.find("RShoulderRoll")->second]->
                m_State.m_rValue[0] = -0.5;

            r_joint_[r_joint_idx_map_.find("LKneePitch")->second]->m_State.m_rValue[0] = 0.3;
            r_joint_[r_joint_idx_map_.find("RKneePitch")->second]->m_State.m_rValue[0] = 0.3;
            r_joint_[r_joint_idx_map_.find("LAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
            r_joint_[r_joint_idx_map_.find("RAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
             break;

        case 1:
            vp_joint_[2]->m_State.m_rValue[0] = 0.376-0.004;
            r_joint_[r_joint_idx_map_.find("LShoulderPitch")->second]->
                m_State.m_rValue[0] = 1.5;
            r_joint_[r_joint_idx_map_.find("RShoulderPitch")->second]->
                m_State.m_rValue[0] = 1.5;
            break;
    }

    KIN_UpdateFrame_All_The_Entity();
}
