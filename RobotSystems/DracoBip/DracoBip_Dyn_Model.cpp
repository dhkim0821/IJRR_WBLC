#include "DracoBip_Dyn_Model.hpp"

#include <Utils/pseudo_inverse.hpp>

using namespace RigidBodyDynamics::Math;

DracoBip_Dyn_Model::DracoBip_Dyn_Model(RigidBodyDynamics::Model* model){
    model_ = model;
}

DracoBip_Dyn_Model::~DracoBip_Dyn_Model(){
}

bool DracoBip_Dyn_Model::getMassInertia(dynacore::Matrix & a){
    a = A_;
    return true;
}

bool DracoBip_Dyn_Model::getInverseMassInertia(dynacore::Matrix & ainv){
    ainv = Ainv_;
    return true;
}

bool DracoBip_Dyn_Model::getGravity(dynacore::Vector &  grav){
    grav = grav_;
    return true;
}
bool DracoBip_Dyn_Model::getCoriolis(dynacore::Vector & coriolis){
    coriolis = coriolis_;
    return true;
}

void DracoBip_Dyn_Model::UpdateDynamics(const dynacore::Vector & q, const dynacore::Vector & qdot){
    // Mass Matrix
    A_ = dynacore::Matrix::Zero(model_->qdot_size, model_->qdot_size);
    CompositeRigidBodyAlgorithm(*model_, q, A_, false);

    // Ainv_ = A_.inverse();
    dynacore::pseudoInverse(A_, 1.e-10, Ainv_, 0);

    dynacore::Vector ZeroQdot = dynacore::Vector::Zero(model_->qdot_size);
    // Gravity
    dynacore::Vector grav_tmp = dynacore::Vector::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, ZeroQdot, ZeroQdot, grav_tmp);
    grav_ = grav_tmp;
    // Coriolis
    dynacore::Vector coriolis_tmp = dynacore::Vector::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, qdot, ZeroQdot, coriolis_tmp);

    coriolis_ = coriolis_tmp - grav_;
} 
