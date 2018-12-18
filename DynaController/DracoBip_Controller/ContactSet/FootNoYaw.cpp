#include "FootNoYaw.hpp"
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip/DracoBip_Definition.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>

FootNoYaw::FootNoYaw(const RobotSystem* robot, int pt):WBDC_ContactSpec(4),
    contact_pt_(pt), max_Fz_(1000.)
{
    idx_Fz_ = 3;
    robot_sys_ = robot;
    sp_ = DracoBip_StateProvider::getStateProvider();
    Jc_ = dynacore::Matrix::Zero(dim_contact_, dracobip::num_qdot);
  // Foot (Local): Ry, X, Y, Z
}

FootNoYaw::~FootNoYaw(){
}
bool FootNoYaw::_UpdateJc(){
    dynacore::Matrix Jtmp, J_local;
    dynacore::Quaternion quat_tmp;

    robot_sys_->getOri(contact_pt_, quat_tmp);
    Eigen::Matrix3d rot_mtx(quat_tmp);
    dynacore::Matrix Ankle_rot_mt(6,6); Ankle_rot_mt.setZero();
    Ankle_rot_mt.topLeftCorner(3,3) = rot_mtx;
    Ankle_rot_mt.bottomRightCorner(3,3) = rot_mtx;

    robot_sys_->getFullJacobian(contact_pt_, Jtmp);
    J_local = Ankle_rot_mt.transpose() * Jtmp;
    Jc_.block(0,0, 1, dracobip::num_qdot) = J_local.block(1, 0, 1, dracobip::num_qdot);
    Jc_.block(1,0, 3, dracobip::num_qdot) = J_local.block(3, 0, 3, dracobip::num_qdot);

    return true;
}
bool FootNoYaw::_UpdateJcDotQdot(){
    dynacore::Vector JcDotQdot_tmp;
    robot_sys_->getFullJDotQdot(contact_pt_, JcDotQdot_tmp);
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    // TODO: we do not consider local frame rotation acceleration
    JcDotQdot_.setZero();
    return true;
}

bool FootNoYaw::_UpdateUf(){
    double mu(0.3);
    double toe(0.07);
    double heel(0.06);

    Uf_ = dynacore::Matrix::Zero(8, dim_contact_);
    // Ry(0), Fx(1), Fy(2), Fz(3)

    // Linear
    Uf_(0, 3) = 1.;  // Fz >= 0

    Uf_(1, 1) = 1.; Uf_(1, 3) = mu;
    Uf_(2, 1) = -1.; Uf_(2, 3) = mu;

    Uf_(3, 2) = 1.; Uf_(3, 3) = mu;
    Uf_(4, 2) = -1.; Uf_(4, 3) = mu;

    // Angular (Flip)
    Uf_(5, 0) = -1/toe; Uf_(5, 3) = 1;
    Uf_(6, 0) = 1/heel; Uf_(6, 3) = 1;

    // Upper bound of vertical directional reaction force
    Uf_(7, 3) = -1.;  // -Fz >= -max_Fz_
    return true;
}

bool FootNoYaw::_UpdateInequalityVector(){
    ieq_vec_ = dynacore::Vector::Zero(8);
    ieq_vec_[7] = -max_Fz_;
    return true;
}
