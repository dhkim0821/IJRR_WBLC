#include "srBlockTerrain.h"

srBlockTerrain::srBlockTerrain(int num_blocks,
                               const Vec3 & location)
  :num_blocks_(num_blocks),
   m_TLink(num_blocks, NULL),
   m_Tjoint(num_blocks, NULL),
   m_Collision(num_blocks, NULL),
   R((50)*0.008f),
   G((20)*0.001f),
   B((10)*0.022f){

  m_BaseLink.SetFrame(EulerZYX(Vec3(0., 0., 0.), location));
  m_BaseLink.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  m_BaseLink.GetGeomInfo().SetDimension(0.00001, 0., 0.);

  this->SetBaseLink(&m_BaseLink);
  this->SetBaseLinkType(FIXED);
  this->SetSelfCollision(true);
}

void srBlockTerrain::_BuildTerrain(const std::vector<double> & xpos_list,
                                   const std::vector<double> & ypos_list,
                                   const std::vector<double> & zpos_list,
                                   const std::vector<double> & l_list,
                                   const std::vector<double> & w_list,
                                   const std::vector<double> & h_list,
                                   const std::vector<Vec3> & ori_list){

  _Set_Link_Shape(l_list, w_list, h_list);
  _ConnectLinks(xpos_list, ypos_list, zpos_list, ori_list);
  _SetCollision();
}

srBlockTerrain::~srBlockTerrain(){
  for(int i(0); i< num_blocks_; ++i){
    if(m_TLink[i] != NULL) delete m_TLink[i];
    if(m_Tjoint[i] != NULL) delete m_Tjoint[i];
    if(m_Collision[i] != NULL) delete m_Collision[i];
  }
}

void srBlockTerrain::_Set_Link_Shape(const std::vector<double> & l,
                                   const std::vector<double> & w,
                                   const std::vector<double> & h){

  for (int idx(0); idx < num_blocks_; ++idx){
    m_TLink[idx] = new srLink();
    m_TLink[idx]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    m_TLink[idx]->GetGeomInfo().SetDimension(Vec3(l[idx],w[idx],h[idx]));
    m_TLink[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(0., 0., 0.),
                                                       Vec3(0., 0., 0)));
    m_TLink[idx]->GetGeomInfo().SetColor(R,G,B, 0.7f);
  }
}

void srBlockTerrain::_ConnectLinks(const std::vector<double> & xpos_list,
                                   const std::vector<double> & ypos_list,
                                   const std::vector<double> & zpos_list,
                                   const std::vector<Vec3> & ori_list){

  bool ori_set(false);

  if(ori_list.size() == num_blocks_) ori_set = true;

  for (int idx(0); idx < num_blocks_; ++idx){
    m_Tjoint[idx] = new srWeldJoint();
    m_Tjoint[idx]->SetParentLink(&m_BaseLink);
    m_Tjoint[idx]->SetChildLink(m_TLink[idx]);
    m_Tjoint[idx]->SetParentLinkFrame(EulerZYX(Vec3(0., 0., 0.),
                                               Vec3(xpos_list[idx],
                                                    ypos_list[idx],
                                                    zpos_list[idx])
                                               ));
    if(ori_set){
      m_Tjoint[idx]->SetChildLinkFrame(EulerZYX(ori_list[idx],
                                                Vec3(0., 0., 0.) ) );
    } else {
      m_Tjoint[idx]->SetChildLinkFrame(EulerZYX(Vec3(0.,0.,0.),
                                                Vec3(0.,0.,0.) ) );
    }
  }
}

void srBlockTerrain::_SetCollision(){
  double fric(0.8);
  double damp(0.05);
  double restit(0.0);

  for(int i(0); i< num_blocks_; ++i){
    m_Collision[i] = new srCollision();
    m_Collision[i]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    m_Collision[i]->GetGeomInfo().SetDimension(m_TLink[i]->GetGeomInfo().GetDimension());
    m_TLink[i]->AddCollision(m_Collision[i]);
    m_TLink[i]->SetFriction(fric);
    m_TLink[i]->SetDamping(damp);
    m_TLink[i]->SetRestitution(restit);
  }
}
