#include "Atlas_Model.hpp"
#include "Atlas_Dyn_Model.hpp"
#include "Atlas_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Configuration.h>
#include <Utils/utilities.hpp>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Atlas_Model::Atlas_Model(){
    model_ = new Model();
    if (!Addons::URDFReadFromFile 
            (THIS_COM"RobotSystems/Atlas/atlas_v3_no_head.urdf", model_, true, false)) {
        std::cerr << "Error loading model atlas_v3_no_head.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new Atlas_Dyn_Model(model_);
    kin_model_ = new Atlas_Kin_Model(model_);

    printf("[Atlas Model] Contructed\n");
}

Atlas_Model::~Atlas_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void Atlas_Model::UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void Atlas_Model::getCentroidInertia(dynacore::Matrix & Icent) const{
    kin_model_->getCentroidInertia(Icent);
}

void Atlas_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const{
    Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

bool Atlas_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const{
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool Atlas_Model::getMassInertia(dynacore::Matrix & A) const {
    return dyn_model_->getMassInertia(A);
}

bool Atlas_Model::getGravity(dynacore::Vector & grav) const {
    return dyn_model_->getGravity(grav);
}

bool Atlas_Model::getCoriolis(dynacore::Vector & coriolis) const{
    return dyn_model_->getCoriolis(coriolis);
}

void Atlas_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
    kin_model_->getJacobian(link_id, J);
}

void Atlas_Model::getFullJDotQdot(int link_id, dynacore::Vector & Jdotqdot) const {
    kin_model_->getJDotQdot(link_id, Jdotqdot);
}

void Atlas_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void Atlas_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}

void Atlas_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}

void Atlas_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void Atlas_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void Atlas_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
    com_pos = kin_model_->com_pos_;
}

void Atlas_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}

void Atlas_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
    centroid_vel = kin_model_->centroid_vel_;
}
