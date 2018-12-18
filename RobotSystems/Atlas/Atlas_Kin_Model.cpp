#include "Atlas_Kin_Model.hpp"
#include "Atlas_Definition.h"
#include <Utils/pseudo_inverse.hpp>
#include <Utils/utilities.hpp>

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

Atlas_Kin_Model::Atlas_Kin_Model( RigidBodyDynamics::Model* model){
    model_ = model;
    Ig_ = dynacore::Matrix::Zero(6,6);
    Jg_ = dynacore::Matrix::Zero(6, model_->qdot_size);
}

Atlas_Kin_Model::~Atlas_Kin_Model(){
}

void Atlas_Kin_Model::UpdateKinematics(const dynacore::Vector & q, const dynacore::Vector & qdot){
    _UpdateCentroidFrame(q, qdot);
}

void Atlas_Kin_Model::_UpdateCentroidFrame_diff(const dynacore::Vector & q, const dynacore::Vector & qdot){
    double mass;
    Vector3d zero_vector;
    zero_vector.setZero();

    Vector3d com_pos;
    Vector3d cm;
    Vector3d link_pos;
    Vector3d p_g;

    getCoMPos(com_pos);
    com_pos_ = com_pos;
    dynacore::pretty_print(com_pos, std::cout, "com_pos");
    dynacore::Matrix Xg_inv = dynacore::Matrix::Zero(6, 6);
    Ig_.setZero();
    dynacore::Matrix Ag = dynacore::Matrix::Zero(6, model_->qdot_size);

    dynacore::Matrix I = dynacore::Matrix::Zero(6, 6);
    dynacore::Matrix Jsp = dynacore::Matrix::Zero(6, model_->qdot_size);

    int start_idx = _find_body_idx(atlas_link::pelvis);
    Matrix3d p;
    Matrix3d cmm;
    Matrix3d R;

    for (int i(start_idx); i<model_->mBodies.size(); ++i){
        R = CalcBodyWorldOrientation(*model_, q, i, false);
        dynacore::pretty_print((dynacore::Matrix)R, std::cout, "Rot");
        //R = R.transpose();
        dynacore::Matrix R_check = R * R.transpose();
        dynacore::Matrix R_check_2 = R.transpose() * R;
        dynacore::Matrix Rmtx(6,6);
        Rmtx.setZero();
        Rmtx.block(0,0, 3,3) = R;
        Rmtx.block(3,3, 3,3) = R;
        cm = model_->mBodies[i].mCenterOfMass;
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, cm, false);
        // Diff computation
        Jsp.setZero();
        CalcPointJacobian6D( *model_, q, i, model_->mBodies[i].mCenterOfMass, 
                Jsp, false);
        Jsp = Rmtx * Jsp;
        mass = model_->mBodies[i].mMass;
        I.setZero();
        I.block(0, 0, 3, 3) = model_->mBodies[i].mInertia; 
        I.block(3, 3, 3, 3) = mass * dynacore::Matrix::Identity(3,3);

        p_g = R * (com_pos - link_pos);
        p << 0.0, -p_g[2], p_g[1],
        p_g[2], 0.0, -p_g[0],
        -p_g[1], p_g[0], 0.0;

        Xg_inv.block(0,0, 3,3) = R;
        Xg_inv.block(3,3, 3,3) = R;
        Xg_inv.block(3,0, 3,3) = p * R;

        dynacore::Matrix Xg_test = Xg_inv.transpose() * Xg_inv;


        Ig_ = Ig_ + Xg_inv.transpose() * I * Xg_inv;
        Ag = Ag + Xg_inv.transpose() * I * Jsp;
        //dynacore::pretty_print(Xg_inv, std::cout, "Xg inv");
        //dynacore::pretty_print(I, std::cout, "I");
        //dynacore::pretty_print(Ig_, std::cout, "Ig sequence");
        //dynacore::pretty_print(Xg_test, std::cout, "Xg test");
        //dynacore::pretty_print(R_check, std::cout, "Rcheck");
        //dynacore::pretty_print(R_check_2, std::cout, "Rcheck_2");
    }
    Jg_ = Ig_.inverse() * Ag;
    dynacore::pretty_print(Ig_, std::cout, "diff_Ig");
    centroid_vel_ = Jg_ * qdot;
    dynacore::Vector h_tot = Ag * qdot;

    dynacore::Vect3 com_vel;
    getCoMVel(com_vel);

    dynacore::pretty_print(h_tot, std::cout, "diff cent momentum");
    dynacore::pretty_print(centroid_vel_, std::cout, "diff cent vel");
    dynacore::pretty_print(com_vel, std::cout, "diff com_vel");
}


void Atlas_Kin_Model::_UpdateCentroidFrame(const dynacore::Vector & q, 
        const dynacore::Vector & qdot){
    //_UpdateCentroidFrame_diff(q, qdot);
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

    int start_idx = _find_body_idx(atlas_link::pelvis);
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
            //Xg_inv.block(3,0, 3,3) = R * p;

            //printf("%d th\n", i);
            //dynacore::pretty_print(Ig_, std::cout, "Ig sequence");
            Ig_ = Ig_ + Xg_inv.transpose() * I * Xg_inv;
            Ag = Ag + Xg_inv.transpose() * I * Jsp;
    }
    Jg_ = Ig_.inverse() * Ag;
    centroid_vel_ = Jg_ * qdot;
    dynacore::Vector h_tot = Ag * qdot;

    //dynacore::Vect3 com_vel;
    //getCoMVel(com_vel);

    //dynacore::pretty_print(h_tot, std::cout, "cent momentum");
    //dynacore::pretty_print(Ig_, std::cout, "Ig");
    //dynacore::pretty_print(centroid_vel_, std::cout, "cent vel");
    //dynacore::pretty_print(com_vel, std::cout, "com_vel");
}


void Atlas_Kin_Model::getCoMJacobian(dynacore::Matrix & Jcom) {
    Vector3d zero_vector = Vector3d::Zero();
    dynacore::Vector q;

    Jcom = dynacore::Matrix::Zero(3, model_->qdot_size);
    MatrixNd J(3, model_->qdot_size);

    double mass;
    double tot_mass(0.0);
    int start_idx = _find_body_idx(atlas_link::pelvis);

    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        // CoM Jacobian Update
        J.setZero();
        CalcPointJacobian(*model_, q, i, model_->mBodies[i].mCenterOfMass, J, false);// TODO: Check whether CoM or Zero vector is correct
        Jcom +=  mass * J;
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}

void Atlas_Kin_Model::getCoMPos(dynacore::Vect3 & CoM_pos) {
    Vector3d zero_vector = Vector3d::Zero();
    dynacore::Vector q;

    CoM_pos.setZero();
    Vector3d link_pos;

    int start_idx = _find_body_idx(atlas_link::pelvis);

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

    //test
    //double rbdl_mass;
    //dynacore::Vector qdot;
    //Math::Vector3d rbdl_com, rbdl_com_vel, rbdl_ang_mom;
    //RigidBodyDynamics::Utils::CalcCenterOfMass(*model_, q, qdot, rbdl_mass, 
    //rbdl_com, &rbdl_com_vel, &rbdl_ang_mom,false);
    //printf("rbdl mass: %f\n", rbdl_mass);
    //dynacore::pretty_print(rbdl_com, std::cout, "rbdl_com");
    //dynacore::pretty_print(rbdl_com_vel, std::cout, "rbdl_com_vel");
    //dynacore::pretty_print(rbdl_ang_mom, std::cout, "rbdl_ angular momentum");
}

void Atlas_Kin_Model::getCoMVel(dynacore::Vect3 & CoM_vel) {
    dynacore::Vector q, qdot;
    int start_idx = _find_body_idx(atlas_link::pelvis);
    CoM_vel = dynacore::Vector::Zero(3);
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

void Atlas_Kin_Model::getPos(int link_id, dynacore::Vect3 & pos){
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

void Atlas_Kin_Model::getOri(int link_id, dynacore::Quaternion & ori){
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

void Atlas_Kin_Model::getLinearVel(int link_id, dynacore::Vect3 & vel){
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

void Atlas_Kin_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel){
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

void Atlas_Kin_Model::getJacobian(int link_id, dynacore::Matrix &J){
    dynacore::Vector q;
    J = dynacore::Matrix::Zero(6, model_->qdot_size);

    unsigned int bodyid = _find_body_idx(link_id);
    Vector3d zero_vector = Vector3d::Zero();

    if(bodyid >=model_->fixed_body_discriminator){
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                J, false);
    }
    else{
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mBodies[bodyid].mCenterOfMass,
                J, false);
    }
}

void Atlas_Kin_Model::getJDotQdot(int link_id, dynacore::Vector & JDotQdot){
    dynacore::Vector q, qdot, qddot; //dummy

    unsigned int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    JDotQdot[5] -= 9.81;
}

unsigned int Atlas_Kin_Model::_find_body_idx(int id) const {
    switch(id){
        case atlas_link::pelvis:
            return model_->GetBodyId("pelvis");
        case atlas_link::torso:
            return model_->GetBodyId("utorso");
        case atlas_link::rightFoot:
            return model_->GetBodyId("r_foot");
        case atlas_link::leftFoot:
            return model_->GetBodyId("l_foot");
        case atlas_link::rightHand:
            return model_->GetBodyId("r_hand");
        case atlas_link::leftHand:
            return model_->GetBodyId("l_hand");
            //case atlas_link::head:
            //return model_->GetBodyId("utorso");
    }
    return (unsigned int)(id+2);
}
