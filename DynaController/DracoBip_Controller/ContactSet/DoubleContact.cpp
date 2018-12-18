#include "DoubleContact.hpp"
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip/DracoBip_Definition.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <Utils/utilities.hpp>

DoubleContact::DoubleContact(const RobotSystem* robot):WBDC_ContactSpec(10)
{
  robot_sys_ = robot;
  sp_ = DracoBip_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(dim_contact_, dracobip::num_qdot);
  // Left (Local): Ry, Rz, X, Y, Z
  // Right (Local): Ry, Rz, X, Y, Z
  
  // printf("[Double Contact] Constructed\n");
}

DoubleContact::~DoubleContact(){}

bool DoubleContact::_UpdateJc(){
  dynacore::Matrix Jtmp, J_local;
  dynacore::Quaternion quat_tmp;

  // Left
  robot_sys_->getOri(dracobip_link::lAnkle, quat_tmp);
  Eigen::Matrix3d l_rot_mt(quat_tmp);
  dynacore::Matrix lAnkle_rot_mt(6,6); lAnkle_rot_mt.setZero();
  lAnkle_rot_mt.topLeftCorner(3,3) = l_rot_mt;
  lAnkle_rot_mt.bottomRightCorner(3,3) = l_rot_mt;

  robot_sys_->getFullJacobian(dracobip_link::lAnkle, Jtmp);
  J_local = lAnkle_rot_mt.transpose() * Jtmp;
  Jc_.block(0, 0, 5, dracobip::num_qdot) = J_local.block(1, 0, 5, dracobip::num_qdot);

  // Right
  robot_sys_->getOri(dracobip_link::rAnkle, quat_tmp);
  Eigen::Matrix3d r_rot_mt(quat_tmp);
  dynacore::Matrix rAnkle_rot_mt(6,6); rAnkle_rot_mt.setZero();
  rAnkle_rot_mt.topLeftCorner(3,3) = r_rot_mt;
  rAnkle_rot_mt.bottomRightCorner(3,3) = r_rot_mt;

  robot_sys_->getFullJacobian(dracobip_link::rAnkle, Jtmp);
  J_local = rAnkle_rot_mt.transpose() * Jtmp;
  Jc_.block(5, 0, 5, dracobip::num_qdot) = J_local.block(1, 0, 5, dracobip::num_qdot);

  // dynacore::pretty_print(Jc_, std::cout, "double] Jc");
  return true;
}
bool DoubleContact::_UpdateJcDotQdot(){
    // TODO
  dynacore::Matrix JcDot(dim_contact_, dracobip::num_qdot);
  dynacore::Matrix jcdot_tmp;

  // dynacore::pretty_print(JcDot, std::cout,  "JcDot");
  JcDot.setZero();
  JcDotQdot_ = JcDot * sp_->Qdot_;
  return true;
}

bool DoubleContact::_UpdateUf(){
  double mu(0.3);
  double toe(0.07);
  double heel(0.06);

  int size_u(11);
  Uf_ = dynacore::Matrix::Zero(size_u*2, dim_contact_);

  dynacore::Matrix U;
  _setU(toe, heel, mu, U);
  
  Uf_.block(0, 0, size_u, 5) = U;
  Uf_.block(size_u, 5, size_u, 5) = U;
  return true;
}

bool DoubleContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(11*2);
  return true;
}

void DoubleContact::_setU(double toe, double heel, double mu, dynacore::Matrix & U){
  U = dynacore::Matrix::Zero(11, 5);
  // Ry, Rz, Fx, Fy, Fz

  // Linear
  U(0, 4) = 1.;

  U(1, 2) = 1.; U(1, 4) = mu;
  U(2, 2) = -1.; U(2, 4) = mu;

  U(3, 3) = 1.; U(3, 4) = mu;
  U(4, 3) = -1.; U(4, 4) = mu;

  // Angular (Flip)
  U(5, 0) = -1/toe; U(5, 4) = 1;
  U(6, 0) = 1/heel; U(6, 4) = 1;

  U(7, 0) = -mu/toe; U(7, 1) = -1/toe; U(7, 3) = -1; U(7, 4) = mu;
  U(8, 0) = -mu/toe; U(8, 1) = 1/toe; U(8, 3) = 1; U(8, 4) = mu;

  U(9, 0) = mu/heel; U(9, 1) = 1/heel; U(9, 3) = -1; U(9, 4) = mu;
  U(10, 0) = mu/heel; U(10, 1) = -1/heel; U(10, 3) = 1; U(10, 4) = mu;

  //dynacore::pretty_print(U, std::cout, "U");
}
