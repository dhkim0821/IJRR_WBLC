#include "Terrain.h"



Terrain::Terrain(){
    // Box 1
    link_[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
    link_[0].GetGeomInfo().SetColor(0.523f, 0.927f, 0.845f, 10.0f);
    link_[0].GetGeomInfo().SetDimension(0.6, 1.5, 0.07);
    link_[0].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.8,0.0, 0.035)));
    link_[0].SetFriction(20.0);
    link_[0].SetDamping(0.7);
    link_[0].SetRestitution(0.0);

    this->SetBaseLink(&link_[0]);
    this->SetBaseLinkType(srSystem::FIXED);
    // this->SetSelfCollision(true);
    
    // Collision
    for (int i(0); i<NUM_STRUCTURE; ++i){
        collision_[i].GetGeomInfo().SetShape(link_[i].GetGeomInfo().GetShape());
        collision_[i].GetGeomInfo().SetDimension(link_[i].GetGeomInfo().GetDimension());
        link_[i].AddCollision(& collision_[i]);
    }
    
}
