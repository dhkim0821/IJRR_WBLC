#include "NAO_Model.hpp"
#include "NAO_Dyn_Model.hpp"
#include "NAO_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Configuration.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

NAO_Model::NAO_Model(){
    model_ = new Model();

     //if (!Addons::URDFReadFromFile (THIS_COM"/RobotSystems/NAO/nao_V50.urdf",
                //model_, false)) {
        //std::cerr << "Error loading model nao_V50.urdf" << std::endl;
        //abort();
    //}
    if (!Addons::URDFReadFromFile (THIS_COM"/RobotSystems/NAO/nao_simple.urdf",
                model_, true, false)) {
        std::cerr << "Error loading model nao_simple.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new NAO_Dyn_Model(model_);
    kin_model_ = new NAO_Kin_Model(model_);

    printf("[NAO Model] Contructed\n");
}

NAO_Model::~NAO_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void NAO_Model::UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void NAO_Model::getCentroidInertia(dynacore::Matrix & Icent) const{
    kin_model_->getCentroidInertia(Icent);
}

void NAO_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const{
    Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

bool NAO_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const{
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool NAO_Model::getMassInertia(dynacore::Matrix & A) const {
    return dyn_model_->getMassInertia(A);
}

bool NAO_Model::getGravity(dynacore::Vector & grav) const {
    return dyn_model_->getGravity(grav);
}

bool NAO_Model::getCoriolis(dynacore::Vector & coriolis) const{
    return dyn_model_->getCoriolis(coriolis);
}

void NAO_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
    J.setZero();
    kin_model_->getJacobian(link_id, J);
}

void NAO_Model::getFullJDotQdot(int link_id, dynacore::Vector & JdotQdot) const {
    kin_model_->getJDotQdot(link_id, JdotQdot);
}

void NAO_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void NAO_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}

void NAO_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}

void NAO_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void NAO_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void NAO_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
    com_pos = kin_model_->com_pos_;
}

void NAO_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}

void NAO_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
    centroid_vel = kin_model_->centroid_vel_;
}
