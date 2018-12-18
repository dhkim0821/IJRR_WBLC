#include "Mercury_Kin_Model.hpp"
#include "Mercury_Definition.h"
#include <Utils/pseudo_inverse.hpp>
#include <Utils/utilities.hpp>


using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

Mercury_Kin_Model::Mercury_Kin_Model( RigidBodyDynamics::Model* model):
    gravity_(9.81)
{
    model_ = model;
    Ig_ = dynacore::Matrix::Zero(6,6);
    Jg_ = dynacore::Matrix::Zero(6, model_->qdot_size);
}

Mercury_Kin_Model::~Mercury_Kin_Model(){
}

void Mercury_Kin_Model::UpdateKinematics(const dynacore::Vector & q, 
        const dynacore::Vector & qdot){
    _UpdateCentroidFrame(q, qdot);
}

void Mercury_Kin_Model::_UpdateCentroidFrame(const dynacore::Vector & q,
        const dynacore::Vector & qdot){
    double mass;
    Vector3d zero_vector;
    zero_vector.setZero();

    Vector3d com_pos;
    Vector3d cm;
    Vector3d link_pos;
    Vector3d p_g;

    getCoMPos(com_pos);
    com_pos_ = com_pos;

    dynacore::Matrix Xg_inv = dynacore::Matrix::Zero(6, 6);
    Ig_.setZero();
    dynacore::Matrix Ag = dynacore::Matrix::Zero(6, model_->qdot_size);

    dynacore::Matrix I = dynacore::Matrix::Zero(6, 6);
    dynacore::Matrix Jsp = dynacore::Matrix::Zero(6, model_->qdot_size);

    int start_idx = _find_body_idx(mercury_link::body);
    Matrix3d p;
    Matrix3d cmm;
    Matrix3d R;

    for (int i(start_idx); i<model_->mBodies.size(); ++i){
        R = CalcBodyWorldOrientation(*model_, q, i, false);

        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, zero_vector, false);

        Jsp.setZero();
        CalcBodySpatialJacobian( *model_, q, i, Jsp, false);

        mass = model_->mBodies[i].mMass;
        I.setZero();
        cm = model_->mBodies[i].mCenterOfMass;
        cmm <<
            0.0, -cm[2], cm[1],
            cm[2], 0.0, -cm[0],
            -cm[1], cm[0], 0.0;
            I.setZero();
            I.block(0, 0, 3, 3) = model_->mBodies[i].mInertia + mass * cmm * cmm.transpose();
            I.block(0,3, 3,3) = mass * cmm;
            I.block(3,0, 3,3) = -mass * cmm;
            I.block(3, 3, 3, 3) = mass * dynacore::Matrix::Identity(3,3);

            p_g = R * (com_pos - link_pos);
            p << 0.0, -p_g[2], p_g[1],
            p_g[2], 0.0, -p_g[0],
            -p_g[1], p_g[0], 0.0;

            Xg_inv.block(0,0, 3,3) = R;
            Xg_inv.block(3,3, 3,3) = R;
            Xg_inv.block(3,0, 3,3) = p * R;
            Ig_ = Ig_ + Xg_inv.transpose() * I * Xg_inv;
            Ag = Ag + Xg_inv.transpose() * I * Jsp;
    }
    Jg_ = Ig_.inverse() * Ag;

    centroid_vel_ = Jg_ * qdot;
}

void Mercury_Kin_Model::getCoMJacobian(dynacore::Matrix & Jcom) const {
    Vector3d zero_vector = Vector3d::Zero();
    dynacore::Vector q;

    Jcom = dynacore::Matrix::Zero(3, model_->qdot_size);
    MatrixNd J(3, model_->qdot_size);

    double mass;
    double tot_mass(0.0);
    int start_idx = _find_body_idx(mercury_link::body);

    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        // CoM Jacobian Update
        J.setZero();
        CalcPointJacobian(*model_, q, i, model_->mBodies[i].mCenterOfMass, J, false);
        Jcom +=  mass * J;
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}

void Mercury_Kin_Model::getCoMPos(dynacore::Vect3 & CoM_pos)const {
    Vector3d zero_vector = Vector3d::Zero();
    dynacore::Vector q;

    CoM_pos.setZero();
    Vector3d link_pos;

    int start_idx = _find_body_idx(mercury_link::body);
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM position Update
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, 
                model_->mBodies[i].mCenterOfMass, false);
        CoM_pos += mass * link_pos;
        tot_mass += mass;
    }
    CoM_pos /= tot_mass;
}

void Mercury_Kin_Model::getCoMVel(dynacore::Vect3 & CoM_vel) const {
    dynacore::Vector q, qdot;
    int start_idx = _find_body_idx(mercury_link::body);
    CoM_vel.setZero();
    Vector3d link_vel;

    Vector3d zero_vector = Vector3d::Zero();
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM velocity Update
        link_vel = CalcPointVelocity ( *model_, q, qdot, i, model_->mBodies[i].mCenterOfMass, false);
        CoM_vel += mass * link_vel;
        tot_mass += mass;
    }
    CoM_vel /= tot_mass;
}

void Mercury_Kin_Model::getPos(int link_id, dynacore::Vect3 & pos){
    Vector3d zero;
    dynacore::Vector q;

    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    pos = CalcBodyToBaseCoordinates(*model_, q, _find_body_idx(link_id), zero, false);

}

void Mercury_Kin_Model::getOri(int link_id, dynacore::Quaternion & ori){
    Matrix3d R;
    dynacore::Vector q;
    R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    ori = R.transpose();

    if(ori.w() < 0.){
        ori.w() *= (-1.);
        ori.x() *= (-1.);
        ori.y() *= (-1.);
        ori.z() *= (-1.);
    }
}

void Mercury_Kin_Model::getLinearVel(int link_id, dynacore::Vect3 & vel){
    Vector3d zero;
    dynacore::Vector q, qdot;

    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    vel = CalcPointVelocity ( *model_, q, qdot, _find_body_idx(link_id), zero, false);
}

void Mercury_Kin_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel){
    unsigned int bodyid = _find_body_idx(link_id);
    dynacore::Vector vel, q, qdot;

    if(bodyid >=model_->fixed_body_discriminator){
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    ang_vel = vel.head(3);
}

void Mercury_Kin_Model::getJacobian(int link_id, dynacore::Matrix &J){
    dynacore::Vector q;
    J = dynacore::Matrix::Zero(6, model_->qdot_size);

    unsigned int bodyid = _find_body_idx(link_id);
    Vector3d zero_vector = Vector3d::Zero();

    if(bodyid >=model_->fixed_body_discriminator){
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mFixedBodies
                [bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                J, false);
    }
    else{
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mBodies[bodyid].mCenterOfMass,
                J, false);
    }
    // Virtual rotation joint axis must be always aligned with the global frame
    // Orientation
    //J.block(0,3,3,3) = dynacore::Matrix::Identity(3,3);
    // Linear
    //dynacore::Vect3 p_link, p_body, dp;
    //getPos(link_id, p_link);
    //getPos(mercury_link::body, p_body);
    //dp = p_link - p_body;
    //dynacore::Matrix p_mtx(3,3);
    //p_mtx(0, 0) = 0.;       p_mtx(0, 1) = dp[2];   p_mtx(0, 2) = -dp[1];
    //p_mtx(1, 0) = -dp[2];   p_mtx(1, 1) = 0.;      p_mtx(1, 2) = dp[0];
    //p_mtx(2, 0) = dp[1];    p_mtx(2, 1) = -dp[0];  p_mtx(2, 2) = 0.;
    //J.block(3,3,3,3) = p_mtx;
}

void Mercury_Kin_Model::getJDotQdot(int link_id, dynacore::Vector & JDotQdot){
    dynacore::Vector q, qdot, qddot;

    unsigned int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mFixedBodies
                [bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    JDotQdot[5] -= gravity_;
}


unsigned int Mercury_Kin_Model::_find_body_idx(int id) const {
    switch(id){
        case mercury_link::body:
            return model_->GetBodyId("body");
        case mercury_link::leftFoot:
            return model_->GetBodyId("lfoot");
        case mercury_link::rightFoot:
            return model_->GetBodyId("rfoot");
        case mercury_link::imu:
            return model_->GetBodyId("imu");

        case mercury_link::LED_BODY_0:
            return model_->GetBodyId("body_led0");
        case mercury_link::LED_BODY_1:
            return model_->GetBodyId("body_led1");
        case mercury_link::LED_BODY_2:
            return model_->GetBodyId("body_led2");

        case mercury_link::LED_RLEG_0:
            return model_->GetBodyId("rleg_led0");
        case mercury_link::LED_RLEG_1:
            return model_->GetBodyId("rleg_led1");
        case mercury_link::LED_RLEG_2:
            return model_->GetBodyId("rleg_led2");
        case mercury_link::LED_RLEG_3:
            return model_->GetBodyId("rleg_led3");
        case mercury_link::LED_RLEG_4:
            return model_->GetBodyId("rleg_led4");


        case mercury_link::LED_LLEG_0:
            return model_->GetBodyId("lleg_led0");
        case mercury_link::LED_LLEG_1:
            return model_->GetBodyId("lleg_led1");
        case mercury_link::LED_LLEG_2:
            return model_->GetBodyId("lleg_led2");
        case mercury_link::LED_LLEG_3:
            return model_->GetBodyId("lleg_led3");
        case mercury_link::LED_LLEG_4:
            return model_->GetBodyId("lleg_led4");

    }
    return (unsigned int)(id + 2);
}
