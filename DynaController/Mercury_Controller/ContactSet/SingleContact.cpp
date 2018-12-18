#include "SingleContact.hpp"
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>

SingleContact::SingleContact(const RobotSystem* robot, int pt):WBLC_ContactSpec(3),
    contact_pt_(pt), max_Fz_(1000.)
{
    robot_sys_ = robot;
    sp_ = Mercury_StateProvider::getStateProvider();
}

SingleContact::~SingleContact(){
}
bool SingleContact::_UpdateJc(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(contact_pt_, Jtmp);
    Jc_ = Jtmp.block(3, 0, 3, mercury::num_qdot);
    return true;
}
bool SingleContact::_UpdateJcDotQdot(){
    dynacore::Vector JcDotQdot_tmp;
    robot_sys_->getFullJDotQdot(contact_pt_, JcDotQdot_tmp);
    JcDotQdot_ = JcDotQdot_tmp.tail(3);
    return true;
}

bool SingleContact::_UpdateUf(){
    double mu (0.3);
    Uf_ = dynacore::Matrix::Zero(6, dim_contact_);
    Uf_(0, 2) = 1.; // Fz >= 0

    Uf_(1, 0) = 1.; Uf_(1, 2) = mu; // Fx >= - mu * Fz
    Uf_(2, 0) = -1.; Uf_(2, 2) = mu; // Fx <= mu * Fz

    Uf_(3, 1) = 1.; Uf_(3, 2) = mu; // Fy >= - mu * Fz
    Uf_(4, 1) = -1.; Uf_(4, 2) = mu; // Fy <=  mu * Fz

    Uf_(5, 2) = -1.; // -Fz >=  -z_max
    return true;
}

bool SingleContact::_UpdateInequalityVector(){
    ieq_vec_ = dynacore::Vector::Zero(6);
    ieq_vec_[5] = -max_Fz_;
    return true;
}
