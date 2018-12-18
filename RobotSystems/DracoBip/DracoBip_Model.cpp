#include "DracoBip_Model.hpp"
#include "DracoBip_Dyn_Model.hpp"
#include "DracoBip_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Configuration.h>
#include <Utils/utilities.hpp>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

DracoBip_Model::DracoBip_Model(){
    model_ = new Model();
    if (!Addons::URDFReadFromFile 
            (THIS_COM"RobotSystems/DracoBip/DracoBip.urdf", model_, true, false)) {
        std::cerr << "Error loading model DracoBip.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new DracoBip_Dyn_Model(model_);
    kin_model_ = new DracoBip_Kin_Model(model_);

    printf("[DracoBip Model] Contructed\n");
}

DracoBip_Model::~DracoBip_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void DracoBip_Model::UpdateSystem(const dynacore::Vector & q, 
        const dynacore::Vector & qdot){
    dynacore::Vector qddot = qdot; qddot.setZero();

    UpdateKinematics(*model_, q, qdot, qddot);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void DracoBip_Model::getCentroidInertia(dynacore::Matrix & Icent) const{
    kin_model_->getCentroidInertia(Icent);
}

void DracoBip_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const{
    Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

bool DracoBip_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const{
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool DracoBip_Model::getMassInertia(dynacore::Matrix & A) const {
    return dyn_model_->getMassInertia(A);
}

bool DracoBip_Model::getGravity(dynacore::Vector & grav) const {
    return dyn_model_->getGravity(grav);
}

bool DracoBip_Model::getCoriolis(dynacore::Vector & coriolis) const{
    return dyn_model_->getCoriolis(coriolis);
}

void DracoBip_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
    kin_model_->getJacobian(link_id, J);
}

void DracoBip_Model::getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const {
    kin_model_->getJDotQdot(link_id, JDotQdot);
}

void DracoBip_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void DracoBip_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}

void DracoBip_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}

void DracoBip_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void DracoBip_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void DracoBip_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
    com_pos = kin_model_->com_pos_;
}

void DracoBip_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    //kin_model_->getCoMVel(com_vel);
    com_vel = (kin_model_->centroid_vel_).tail(3);
}

void DracoBip_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
    centroid_vel = kin_model_->centroid_vel_;
}
