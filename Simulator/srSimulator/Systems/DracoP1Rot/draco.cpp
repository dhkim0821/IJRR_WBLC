#include "draco.h"

// #define FIX_IN_AIR 

srDraco::srDraco(Vec3 location):
  SystemGenerator()
{
  link_idx_map_.clear();
  r_joint_idx_map_.clear();

  _AssembleRobot(location);
  _SetInitialConf();
  _SetCollision();
  _SetJointLimit();

  num_act_joint_ = 3;

  printf("[DRACO] ASSEMBLED\n");
}

srDraco::~srDraco(){
}


void srDraco::_AssembleRobot(Vec3 location){
  // Link: 1) vertical base
  //       2) horizontal
  //       3) rotation
  //       4) body
  //       5) thigh
  //       6) shank
  //       7) foot
  link_.resize(7);
  _DefineLinks();
  _SetInertia();

  // Virtual Joint: 1) z
  //                2) x
  //                3) Ry
  vp_joint_.resize(2);
  vr_joint_.resize(1);
  _DefineVirtualJoint();

  // Joint: 1) Body Pitch
  //        2) Knee
  //        3) Ankle
  //Build up and down fixture
  r_joint_.resize(3);
  _DefineActuatedJoint();

  // Basement Setting
  link_[0]->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), location));
  this->SetBaseLink(link_[0]);
  this->SetBaseLinkType(srSystem::FIXED);
  this->SetSelfCollision(true);
}

void srDraco::_SetJointLimit(){
}
void srDraco::_SetInertia(){
  Inertia body_inertia = Inertia(0.25,0.268, 0.114, -0.002, 0.001, 0.006);
  body_inertia.SetMass(11.313);
  link_[link_idx_map_.find("body")->second]->SetInertia(body_inertia);

  Inertia thigh_inertia = Inertia(0.0777, 0.0761, 0.0061, -0.0001, 0.0084, -0.0005);
  thigh_inertia.SetMass(5.4818);
  link_[link_idx_map_.find("upperLeg")->second]->SetInertia(thigh_inertia);

  Inertia shank_inertia = Inertia(0.0777, 0.0761, 0.0061, -0.0001, 0.0084, -0.0005);
  shank_inertia.SetMass(3.7701);
  link_[link_idx_map_.find("lowerLeg")->second]->SetInertia(shank_inertia);

  Inertia foot_inertia = Inertia(0.001766, 0.0020957, 0.0004916, -0.0000003, -0.0001134, 0.0000006);
  foot_inertia.SetMass(0.6817573);
  link_[link_idx_map_.find("foot")->second]->SetInertia(foot_inertia);

}

void srDraco::_SetCollision(){
  collision_.resize(1);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
    collision_[i]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    collision_[i]->GetGeomInfo().SetDimension(0.2, 0.07, 0.02);
    collision_[i]->SetLocalFrame(EulerZYX(Vec3(0.,0.,0.), Vec3(0.02, 0., -0.01)));
  }

  link_[link_idx_map_.find("foot")->second]->AddCollision(collision_[0]);

  double fric(0.3);
  link_[link_idx_map_.find("foot")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("foot")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("foot")->second]->SetRestitution(restit);
}

void srDraco::_SetInitialConf(){
#ifdef FIX_IN_AIR
  vp_joint_[0]->m_State.m_rValue[0] = 0.87 - 0.19  + 0.2;
#else
  vp_joint_[0]->m_State.m_rValue[0] = 0.87 - 0.19;
#endif

  vp_joint_[1]->m_State.m_rValue[0] = 0.01;

  r_joint_[r_joint_idx_map_.find("bodyPitch")->second]->m_State.m_rValue[0] = -1.;
  r_joint_[r_joint_idx_map_.find("kneePitch")->second]->m_State.m_rValue[0] = 2.;
  r_joint_[r_joint_idx_map_.find("ankle")->second]->m_State.m_rValue[0] = -1.;
  KIN_UpdateFrame_All_The_Entity();
}


void srDraco::_DefineLinks(){
  Inertia dummy = Inertia(0.0);

  int idx(0);
  // Vertical Base
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("vertical_base", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  link_[idx]->GetGeomInfo().SetFileName(ModelPath"meshes/supportStructure_base.3ds");
  link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF,0., SR_PI),Vec3(-0.5,-0.4,-0.06)));

  // horizontal moving stage
  ++idx;
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("horizontal_stage", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  link_[idx]->GetGeomInfo().SetFileName(ModelPath"meshes/horiStage.3ds");
  link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.,0., 0.),Vec3(-0.25,0.,0.115)));
  link_[idx]->SetInertia(dummy);

  // Rotating stage
  ++idx;
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("rotating_stage", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  // link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.,0., 0.),Vec3(-0.25,0.,0.115)));
  link_[idx]->SetInertia(dummy);

  // Body
  ++idx;
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("body", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  link_[idx]->GetGeomInfo().SetFileName(ModelPath"meshes/body.3ds");
  link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0.,0., SR_PI),Vec3(-0.28,0.139, -0.115)));

  // Thigh
  ++idx;
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("upperLeg", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  link_[idx]->GetGeomInfo().SetFileName(ModelPath"meshes/upperLeg.3ds");
  link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0., -SR_PI_HALF - 0.0),Vec3(-0.078, -0.087,0.2773)));

  // Shank
  ++idx;
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("lowerLeg", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  link_[idx]->GetGeomInfo().SetFileName(ModelPath"meshes/lowerLeg.3ds");
  link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.,SR_PI_HALF),Vec3(0.0088 - 0.075, 0.055, -0.255)));


  // Foot
  ++idx;
  link_[idx] = new srLink();
  link_idx_map_.insert(make_pair("foot", idx));
  link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  link_[idx]->GetGeomInfo().SetFileName(ModelPath"meshes/foot.3ds");
  link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI_HALF, 0., SR_PI),Vec3(-0.08, -0.048, -0.013)));
}


void srDraco::_DefineVirtualJoint(){
  ///////////// Joint
  int jidx(0);
  // Z - Virtual Joint
  vp_joint_[jidx] = new srPrismaticJoint();
  vp_joint_[jidx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);

#ifdef FIX_IN_AIR
  vp_joint_[jidx]->SetActType(srJoint::HYBRID);
#else
  vp_joint_[jidx]->SetActType(srJoint::TORQUE);
#endif

  vp_joint_[jidx]->SetParentLink(link_[0]);
  vp_joint_[jidx]->SetParentLinkFrame(EulerZYX(Vec3(0.,0.,0.), Vec3(0., -0.3, 0.)));
  vp_joint_[jidx]->SetChildLink(link_[1]);

  // X - Virtual Joint
  ++jidx;
  vp_joint_[jidx] = new srPrismaticJoint();
  vp_joint_[jidx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);

#ifdef FIX_IN_AIR
  vp_joint_[jidx]->SetActType(srJoint::HYBRID);
#else
  vp_joint_[jidx]->SetActType(srJoint::TORQUE);
#endif

  vp_joint_[jidx]->SetParentLink(link_[1]);
  vp_joint_[jidx]->SetParentLinkFrame(EulerZYX(Vec3(0.,SR_PI_HALF,0.), Vec3(0., 0.0, 0.)));
  vp_joint_[jidx]->SetChildLink(link_[2]);
  vp_joint_[jidx]->SetChildLinkFrame(EulerZYX(Vec3(0., SR_PI_HALF,0.), Vec3(0., 0.0, 0.)));

  // Ry - Virtual Joint
  jidx = 0;
  vr_joint_[jidx] = new srRevoluteJoint();
  vr_joint_[jidx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);

#ifdef FIX_IN_AIR
  vr_joint_[jidx]->SetActType(srJoint::HYBRID);
#else
  vr_joint_[jidx]->SetActType(srJoint::TORQUE);
#endif

  vr_joint_[jidx]->SetParentLink(link_[2]);
  vr_joint_[jidx]->SetParentLinkFrame(EulerZYX(Vec3(0.,0., -SR_PI_HALF), Vec3(0., 0.0, 0.)));
  vr_joint_[jidx]->SetChildLink(link_[3]);
  vr_joint_[jidx]->SetChildLinkFrame(EulerZYX(Vec3(0., 0., -SR_PI_HALF), Vec3(0., -0.28, 0.)));

}

void srDraco::_DefineActuatedJoint(){
    // Body Pitch
  int jidx(0);
  r_joint_[jidx] = new srRevoluteJoint();
  r_joint_idx_map_.insert(make_pair("bodyPitch", jidx));
  r_joint_[jidx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  r_joint_[jidx]->SetActType(srJoint::TORQUE);
  r_joint_[jidx]->SetParentLink(link_[3]);
  r_joint_[jidx]->SetParentLinkFrame(EulerZYX(Vec3(0.,0.,-SR_PI_HALF), Vec3(0.007, -0.007, -0.115)));
  r_joint_[jidx]->SetChildLink(link_[4]); // Upper Leg
  r_joint_[jidx]->SetChildLinkFrame(EulerZYX(Vec3(0., 0.,-SR_PI_HALF), Vec3(-0.0061, 0., 0.2473)));

  // Knee
  ++jidx;
  r_joint_[jidx] = new srRevoluteJoint();
  r_joint_idx_map_.insert(make_pair("kneePitch", jidx));
  r_joint_[jidx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  r_joint_[jidx]->SetActType(srJoint::TORQUE);
  r_joint_[jidx]->SetParentLink(link_[4]);
  r_joint_[jidx]->SetParentLinkFrame(EulerZYX(Vec3(0.,0.,-SR_PI_HALF), Vec3(-0.0061, 0., -0.49 + 0.2473)));
  r_joint_[jidx]->SetChildLink(link_[5]); // Lower Leg
  r_joint_[jidx]->SetChildLinkFrame(EulerZYX(Vec3(0., 0.,-SR_PI_HALF), Vec3(0.0038, 0.0022, 0.2647)));

  // Ankle
  ++jidx;
  r_joint_[jidx] = new srRevoluteJoint();
  r_joint_idx_map_.insert(make_pair("ankle", jidx));
  r_joint_[jidx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  r_joint_[jidx]->SetActType(srJoint::TORQUE);
  r_joint_[jidx]->SetParentLink(link_[5]);
  r_joint_[jidx]->SetParentLinkFrame(EulerZYX(Vec3(0.,0.,-SR_PI_HALF), Vec3(0.0038, 0.0022, -0.49 + 0.2647)));
  r_joint_[jidx]->SetChildLink(link_[6]); // Lower Leg
  r_joint_[jidx]->SetChildLinkFrame(EulerZYX(Vec3(0., 0.,-SR_PI_HALF), Vec3(-0.0183, 0.0, 0.0232)));
}
