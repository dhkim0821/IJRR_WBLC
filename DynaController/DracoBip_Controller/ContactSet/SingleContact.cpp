#include "SingleContact.hpp"
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip/DracoBip_Definition.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>

SingleContact::SingleContact(const RobotSystem* robot, int pt):WBLC_ContactSpec(5),
    contact_pt_(pt), max_Fz_(1000.)
{
    idx_Fz_ = 4;
    robot_sys_ = robot;
    sp_ = DracoBip_StateProvider::getStateProvider();
  // Foot (Local): Ry, Rz, X, Y, Z
}

SingleContact::~SingleContact(){
}
bool SingleContact::_UpdateJc(){
    dynacore::Matrix Jtmp, J_local;
    dynacore::Quaternion quat_tmp;

    robot_sys_->getOri(contact_pt_, quat_tmp);
    Eigen::Matrix3d rot_mtx(quat_tmp);
    dynacore::Matrix Ankle_rot_mt(6,6); Ankle_rot_mt.setZero();
    Ankle_rot_mt.topLeftCorner(3,3) = rot_mtx;
    Ankle_rot_mt.bottomRightCorner(3,3) = rot_mtx;

    robot_sys_->getFullJacobian(contact_pt_, Jtmp);
    J_local = Ankle_rot_mt.transpose() * Jtmp;
    Jc_ = J_local.block(1, 0, 5, dracobip::num_qdot);

    return true;
}
bool SingleContact::_UpdateJcDotQdot(){
    dynacore::Vector JcDotQdot_tmp;
    robot_sys_->getFullJDotQdot(contact_pt_, JcDotQdot_tmp);
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    // TODO: we do not consider local frame rotation acceleration
    JcDotQdot_.setZero();
    return true;
}

bool SingleContact::_UpdateUf(){
    double mu(0.3);
    double toe(0.07);
    double heel(0.06);

    Uf_ = dynacore::Matrix::Zero(12, dim_contact_);
    // Ry, Rz, Fx, Fy, Fz

    // Linear
    Uf_(0, 4) = 1.;  // Fz >= 0

    Uf_(1, 2) = 1.; Uf_(1, 4) = mu;
    Uf_(2, 2) = -1.; Uf_(2, 4) = mu;

    Uf_(3, 3) = 1.; Uf_(3, 4) = mu;
    Uf_(4, 3) = -1.; Uf_(4, 4) = mu;

    // Angular (Flip)
    Uf_(5, 0) = -1/toe; Uf_(5, 4) = 1;
    Uf_(6, 0) = 1/heel; Uf_(6, 4) = 1;

    Uf_(7, 0) = -mu/toe; Uf_(7, 1) = -1/toe; Uf_(7, 3) = -1; Uf_(7, 4) = mu;
    Uf_(8, 0) = -mu/toe; Uf_(8, 1) = 1/toe; Uf_(8, 3) = 1; Uf_(8, 4) = mu;

    Uf_(9, 0) = mu/heel; Uf_(9, 1) = 1/heel; Uf_(9, 3) = -1; Uf_(9, 4) = mu;
    Uf_(10, 0) = mu/heel; Uf_(10, 1) = -1/heel; Uf_(10, 3) = 1; Uf_(10, 4) = mu;

    // Upper bound of vertical directional reaction force
    Uf_(11, 4) = -1.;  // -Fz >= -max_Fz_
    return true;
}

bool SingleContact::_UpdateInequalityVector(){
    ieq_vec_ = dynacore::Vector::Zero(12);
    ieq_vec_[11] = -max_Fz_;
    return true;
}
