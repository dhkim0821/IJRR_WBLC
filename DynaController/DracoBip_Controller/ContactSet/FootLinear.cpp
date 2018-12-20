#include "FootLinear.hpp"
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip/DracoBip_Definition.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>

FootLinear::FootLinear(const RobotSystem* robot, int pt):WBLC_ContactSpec(3),
    contact_pt_(pt), max_Fz_(1000.)
{
    idx_Fz_ = 2;
    robot_sys_ = robot;
    sp_ = DracoBip_StateProvider::getStateProvider();
    Jc_ = dynacore::Matrix::Zero(dim_contact_, dracobip::num_qdot);
  // Foot (Local): Ry, X, Y, Z
}

FootLinear::~FootLinear(){
}
bool FootLinear::_UpdateJc(){
    dynacore::Matrix Jtmp;

    robot_sys_->getFullJacobian(contact_pt_, Jtmp);
    Jc_ = Jtmp.block(3, 0, 3, dracobip::num_qdot);

    return true;
}

bool FootLinear::_UpdateJcDotQdot(){
    dynacore::Vector JcDotQdot_tmp;
    robot_sys_->getFullJDotQdot(contact_pt_, JcDotQdot_tmp);
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    // TODO: we do not consider local frame rotation acceleration
    JcDotQdot_.setZero();
    return true;
}

bool FootLinear::_UpdateUf(){
    double mu(0.3);
    double toe(0.07);
    double heel(0.06);

    Uf_ = dynacore::Matrix::Zero(6, dim_contact_);
    // Fx(1), Fy(2), Fz(3)

    // Linear
    Uf_(0, 2) = 1.;  // Fz >= 0

    Uf_(1, 0) = 1.; Uf_(1, 2) = mu;
    Uf_(2, 0) = -1.; Uf_(2, 2) = mu;

    Uf_(3, 1) = 1.; Uf_(3, 2) = mu;
    Uf_(4, 1) = -1.; Uf_(4, 2) = mu;

    // Upper bound of vertical directional reaction force
    Uf_(5, 2) = -1.;  // -Fz >= -max_Fz_
    return true;
}

bool FootLinear::_UpdateInequalityVector(){
    ieq_vec_ = dynacore::Vector::Zero(6);
    ieq_vec_[5] = -max_Fz_;
    return true;
}
