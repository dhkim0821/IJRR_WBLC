#include "new_valkyrie.h"

New_Valkyrie::New_Valkyrie():
  SystemGenerator()
{
      //BuildRobot(Vec3(0., 0., 0.), srSystem::FIXED, srJoint::TORQUE,
          //ModelPath"Valkyrie/valkyrie.urdf");
 
  printf("[Valkyrie] ASSEMBLED\n");
}

New_Valkyrie::~New_Valkyrie(){
}

void New_Valkyrie::_SetJointLimit(){
  // r_joint_[r_joint_idx_map_.find("leftShoulderRoll" )->second]->SetPositionLimit(-90, 0);
  // r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->SetPositionLimit(0, 90);

  // r_joint_[r_joint_idx_map_.find("leftElbowPitch" )->second]->SetPositionLimit(-60., 0.);
  // r_joint_[r_joint_idx_map_.find("rightElbowPitch" )->second]->SetPositionLimit(-0., 60.);

  // r_joint_[r_joint_idx_map_.find("leftShoulderPitch" )->second]->SetPositionLimit(-45, 75);
  // r_joint_[r_joint_idx_map_.find("rightShoulderPitch" )->second]->SetPositionLimit(-75, 45);

  // r_joint_[r_joint_idx_map_.find("torsoYaw" )->second]->SetPositionLimit(-140., 140.);


  // for(int i(0);i <3; ++i){
  //   vp_joint_[i]->MakePositionLimit(false);
  //   vr_joint_[i]->MakePositionLimit(false);
  // }
}

void New_Valkyrie::_SetCollision(){
  collision_.resize(2);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[0]->GetGeomInfo().SetDimension(0.27, 0.16, 0.09);
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[1]->GetGeomInfo().SetDimension(0.27, 0.16, 0.09);

  link_[link_idx_map_.find("rightFoot")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("leftFoot")->second]->AddCollision(collision_[1]);

  double fric(0.8);
  link_[link_idx_map_.find("rightFoot")->second]->SetFriction(fric);
  link_[link_idx_map_.find("leftFoot")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("rightFoot")->second]->SetDamping(damp);
  link_[link_idx_map_.find("leftFoot")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("rightFoot")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("leftFoot")->second]->SetRestitution(restit);
}

void New_Valkyrie::_SetInitialConf(){
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

  r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
  r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
  r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
  r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
  r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
  r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
  r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

  r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
  r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
  r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
  r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

  r_joint_[r_joint_idx_map_.find("leftShoulderPitch" )->second]->
      m_State.m_rValue[0] = 0.1;
  r_joint_[r_joint_idx_map_.find("leftShoulderRoll"  )->second]->
      m_State.m_rValue[0] = -1.0;
  r_joint_[r_joint_idx_map_.find("leftElbowPitch"    )->second]->
      m_State.m_rValue[0] = -1.5;
  r_joint_[r_joint_idx_map_.find("leftForearmYaw" )->second]->
      m_State.m_rValue[0] = 0.8;

  switch(pose){
  case 0:

    break;

  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 1.131;

    r_joint_[r_joint_idx_map_.find("rightHipRoll" )->second]->m_State.m_rValue[0] = 0.17;
    r_joint_[r_joint_idx_map_.find("rightAnkleRoll" )->second]->m_State.m_rValue[0] = -0.17;
    r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 1.0;
    r_joint_[r_joint_idx_map_.find("leftHipPitch" )->second]->m_State.m_rValue[0] = -1.0;

    break;

  case 2:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079;

    r_joint_[r_joint_idx_map_.find("leftHipYaw" )->second]->m_State.m_rValue[0] =  0.001277;
    r_joint_[r_joint_idx_map_.find("leftHipRoll" )->second]
        ->m_State.m_rValue[0] =  0.000066;
    r_joint_[r_joint_idx_map_.find("leftHipPitch" )->second]->m_State.m_rValue[0] =  -0.421068;
    r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.915849;
    r_joint_[r_joint_idx_map_.find("leftAnklePitch" )->second]->m_State.m_rValue[0] =  -0.470756;
    r_joint_[r_joint_idx_map_.find("leftAnkleRoll" )->second]->m_State.m_rValue[0] =  0.000312;

    r_joint_[r_joint_idx_map_.find("rightHipYaw" )->second]->m_State.m_rValue[0] =  0.003021;
    r_joint_[r_joint_idx_map_.find("rightHipRoll" )->second]
        ->m_State.m_rValue[0] =  -0.000109;
    r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] =  -0.421119;
    r_joint_[r_joint_idx_map_.find("rightKneePitch" )->second]->m_State.m_rValue[0] = 0.917231;
    r_joint_[r_joint_idx_map_.find("rightAnklePitch" )->second]->m_State.m_rValue[0] =  -0.467243;
    r_joint_[r_joint_idx_map_.find("rightAnkleRoll" )->second]->m_State.m_rValue[0] =  0.000072;
    break;

  case 3:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.032720;
    vp_joint_[2]->m_State.m_rValue[0] =  1.050418;

    r_joint_[r_joint_idx_map_.find("leftHipYaw" )->second]->m_State.m_rValue[0] =  -0.002063;
    r_joint_[r_joint_idx_map_.find("leftHipRoll" )->second]->m_State.m_rValue[0] =  -0.012110;
    r_joint_[r_joint_idx_map_.find("leftHipPitch" )->second]->m_State.m_rValue[0] =  -0.375930;
    r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 1.027675;
    r_joint_[r_joint_idx_map_.find("leftAnklePitch" )->second]->m_State.m_rValue[0] =  -0.663891;
    r_joint_[r_joint_idx_map_.find("leftAnkleRoll" )->second]->m_State.m_rValue[0] =  0.008284;

    r_joint_[r_joint_idx_map_.find("rightHipYaw" )->second]->m_State.m_rValue[0] =  0.017352;
    r_joint_[r_joint_idx_map_.find("rightHipRoll" )->second]->m_State.m_rValue[0] =  -0.058997;
    r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] =  -0.755560;
    r_joint_[r_joint_idx_map_.find("rightKneePitch" )->second]->m_State.m_rValue[0] = 0.650955;
    r_joint_[r_joint_idx_map_.find("rightAnklePitch" )->second]->m_State.m_rValue[0] =  0.106764;
    r_joint_[r_joint_idx_map_.find("rightAnkleRoll" )->second]->m_State.m_rValue[0] =  0.055034;
    break;

  case 4:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079;

    r_joint_[r_joint_idx_map_.find("leftHipYaw" )->second]->m_State.m_rValue[0] =  0.001277;
    r_joint_[r_joint_idx_map_.find("leftHipRoll" )->second]->m_State.m_rValue[0] =  0.000066;
    r_joint_[r_joint_idx_map_.find("leftHipPitch" )->second]->m_State.m_rValue[0] =  -0.421068;
    r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.915849;
    r_joint_[r_joint_idx_map_.find("leftAnklePitch" )->second]->m_State.m_rValue[0] =  -0.470756;
    r_joint_[r_joint_idx_map_.find("leftAnkleRoll" )->second]->m_State.m_rValue[0] =  0.000312;

    r_joint_[r_joint_idx_map_.find("rightHipYaw" )->second]->m_State.m_rValue[0] =  0.003021;
    r_joint_[r_joint_idx_map_.find("rightHipRoll" )->second]->m_State.m_rValue[0] =  -0.000109;
    r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] =  -0.421119;
    r_joint_[r_joint_idx_map_.find("rightKneePitch" )->second]->m_State.m_rValue[0] = 0.917231;
    r_joint_[r_joint_idx_map_.find("rightAnklePitch" )->second]->m_State.m_rValue[0] =  -0.467243;
    r_joint_[r_joint_idx_map_.find("rightAnkleRoll" )->second]->m_State.m_rValue[0] =  0.000072;

    r_joint_[r_joint_idx_map_.find("leftElbowPitch" )->second]->m_State.m_rValue[0] =  -1.4;
    r_joint_[r_joint_idx_map_.find("rightElbowPitch" )->second]->m_State.m_rValue[0] =  1.7;

    break;
  }
  KIN_UpdateFrame_All_The_Entity();
}
