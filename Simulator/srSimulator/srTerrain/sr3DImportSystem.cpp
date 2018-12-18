#include "sr3DImportSystem.h"
#include <srConfiguration.h>

sr3DImportSystem::sr3DImportSystem(string modelnamepath,
                                   const Vec3 & location,
                                   const Vec3 & orientation,
                                   srJoint::ACTTYPE joint_type): m_Link(1, NULL),
                                                                 m_VLink(6, NULL),
                                                                 m_VPjoint(3, NULL),
                                                                 m_VRjoint(3, NULL),
                                                                 m_Collision(1, NULL),
                                                                 R((100)*0.008f),
                                                                 G((60)*0.011f),
                                                                 B((30)*0.012f){
  for(int i(0); i<3; ++i){
    m_VLink[i] = new srLink();
    m_VLink[i+3] = new srLink();
    m_VPjoint[i] = new srPrismaticJoint();
    m_VRjoint[i] = new srRevoluteJoint();
  }
  m_Link[0] = new srLink();
  m_Collision[0] = new srCollision();
  modelnamepath_ = (ObstaclePath + modelnamepath).c_str();
  joint_type_ = joint_type;
  ori_ = orientation;

  // m_VLink[0]->SetFrame(EulerZYX(orientation, location));
  m_VLink[0]->SetFrame(EulerZYX(Vec3(0,0,0), Vec3(0,0,0)));
  this->SetBaseLink(m_VLink[0]);
  this->SetBaseLinkType(FIXED);
  // this->SetSelfCollision(true);
  this->SetSelfCollision(false);

  //test
  _BuildTerrain();
}

sr3DImportSystem::~sr3DImportSystem(){
  delete m_Link[0];

  for(int i(0); i< 3; ++i){
    delete m_VLink[i];
    delete m_VLink[i+3];
    if(m_VPjoint[i] != NULL) delete m_VPjoint[i];
    if(m_VRjoint[i] != NULL) delete m_VRjoint[i];
    if(m_Collision[i] != NULL) delete m_Collision[i];
  }
}

void sr3DImportSystem::_BuildTerrain(){
  _Set_Virtual_Link_Joint();
  _Set_Link_Shape();
  //_SetCollision();
  _Set_Initial_Conf();
}

void sr3DImportSystem::_Set_Initial_Conf(){
  for (int i(0); i<3; ++i){
    m_VPjoint[i]->m_State.m_rValue[0] = 0.0;
    m_VRjoint[i]->m_State.m_rValue[0] = 0.0;

    m_VPjoint[i]->m_State.m_rValue[1] = 0.0;
    m_VRjoint[i]->m_State.m_rValue[1] = 0.0;

    m_VPjoint[i]->m_State.m_rValue[2] = 0.0;
    m_VRjoint[i]->m_State.m_rValue[2] = 0.0;

  }
  KIN_UpdateFrame_All_The_Entity();
}

void sr3DImportSystem::_Set_Virtual_Link_Joint(){

  double passive_radius(0.00001);
  double passive_length(0.00001);

  for(int i(0); i<3; ++i){
    m_VPjoint[i]->SetParentLink(m_VLink[i]);
    m_VPjoint[i]->SetChildLink(m_VLink[i+1]);

    m_VPjoint[i]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_VPjoint[i]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
  }

  for(int i(0); i<3; ++i){
    m_VRjoint[i]->SetParentLink(m_VLink[i+3]);
    if(i == 2)
      m_VRjoint[i]->SetChildLink(m_Link[0]);
    else
      m_VRjoint[i]->SetChildLink(m_VLink[i+4]);

    m_VRjoint[i]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_VRjoint[i]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
  }

  m_VPjoint[0]->SetParentLinkFrame(EulerZYX(Vec3(0., SR_PI_HALF, 0.), Vec3(0., 0., 0.)));
  m_VPjoint[0]->SetChildLinkFrame(EulerZYX(Vec3(0., 0., SR_PI_HALF), Vec3(0., 0., 0.)));

  m_VPjoint[1]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.), Vec3(0., 0., 0.)));
  m_VPjoint[1]->SetChildLinkFrame(EulerZYX(Vec3(0., SR_PI_HALF, 0.), Vec3(0., 0., 0.)));

  m_VPjoint[2]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.), Vec3(0., 0., 0.)));
  m_VPjoint[2]->SetChildLinkFrame(EulerZYX(Vec3(0., 0., -SR_PI_HALF), Vec3(0., 0., 0.)));

  m_VRjoint[0]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.), Vec3(0., 0., 0.)));
  m_VRjoint[0]->SetChildLinkFrame(EulerZYX(Vec3(0., -SR_PI_HALF, 0.), Vec3(0., 0., 0.)));

  m_VRjoint[1]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.), Vec3(0., 0., 0.)));
  m_VRjoint[1]->SetChildLinkFrame(EulerZYX(Vec3(0., 0., SR_PI_HALF), Vec3(0., 0., 0.)));

  // m_VRjoint[2]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.), Vec3(0., 0., 0.)));
  // m_VRjoint[2]->SetChildLinkFrame(EulerZYX(Vec3(SR_PI, 0., 0.), Vec3(0., 0., 0.)));
  m_VRjoint[2]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.), Vec3(0., 0., 0.)));
  m_VRjoint[2]->SetChildLinkFrame(EulerZYX(Vec3(0., 0., -SR_PI_HALF), Vec3(0., 0., 0.)));


  for(int i(0); i<3; ++i){
    m_VLink[i]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_VLink[i]->GetGeomInfo().SetDimension(0.001, 0.001, 0);
  }

  for(int i(0); i<3; ++i) {
    m_VPjoint[i]->SetActType(joint_type_);
    m_VRjoint[i]->SetActType(joint_type_);
  }

  Inertia dummy = Inertia(0.0);
  for(int i(0); i<6; ++i){
    m_VLink[i]->SetInertia(dummy);
  }
}

void sr3DImportSystem::_Set_Link_Shape(){
  Inertia mass = Inertia(0.1);
  Inertia inertia = Inertia(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
  inertia.SetMass(10.3);

  // m_Link[0]->SetInertia(inertia);
  // m_Link[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  // m_Link[0]->GetGeomInfo().SetDimension(Vec3(0.455, 0.277, 0.17));

  m_Link[0]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
  m_Link[0]->GetGeomInfo().SetFileName(modelnamepath_);
  // m_Link[0]->GetGeomInfo().SetColor((100)*0.9f,(60)*0.005f,(30)*0.005f,0.7f);
}

void sr3DImportSystem::_SetCollision(){
  double fric(0.8);
  double damp(0.01);
  double restit(0.0);

    m_Collision[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    m_Collision[0]->GetGeomInfo().SetDimension(0.1,0.1,0.1);
    m_Link[0]->AddCollision(m_Collision[0]);
    m_Link[0]->SetFriction(fric);
    m_Link[0]->SetDamping(damp);
    m_Link[0]->SetRestitution(restit);
  }
