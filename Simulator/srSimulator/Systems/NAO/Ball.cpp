#include "Ball.h"


Ball::Ball(double x, double y, double z, double d, double mass, double rest){
    for( int i(0); i<NUM_BALL; ++i){
        ball_.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
        ball_.GetGeomInfo().SetColor(0.324f, 0.12f, 0.12f);
        //ball_.GetGeomInfo().SetDimension(d, 0.0, 0.0);
        ball_.GetGeomInfo().SetDimension(d, 1.0, 2.0);
        ball_.SetFriction(0.1);
        ball_.SetDamping(0.001);
        ball_.SetRestitution(rest);
    }
    ball_.m_Inertia.SetMass(mass);
    ball_.SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(x, y, z) ) );

    // Collision
    for (int i(0); i<NUM_BALL; ++i){
        collision_.GetGeomInfo().SetShape(ball_.GetGeomInfo().GetShape());
        collision_.GetGeomInfo().SetDimension(ball_.GetGeomInfo().GetDimension());
        ball_.AddCollision(& collision_);
    }
    this->SetBaseLink(&ball_);
    this->SetBaseLinkType(srSystem::DYNAMIC);

}
