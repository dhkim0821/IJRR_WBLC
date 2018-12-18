#include "DracoBip_StateEstimator.hpp"
#include <Utils/utilities.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

void DracoBip_StateEstimator::_RBDL_TEST(){
    // TEST
    dynacore::Vector q(dracobip::num_q); q.setZero();
    dynacore::Vector qdot(dracobip::num_qdot); qdot.setZero();

    q[dracobip_joint::virtual_Rw] = 1.0;
    q[dracobip_joint::lHipYaw] = 1.0;
    //q[dracobip_joint::lHipYaw] = 1.0;
    q[dracobip_joint::virtual_Rw] = 1.0;
    q[dracobip_joint::lAnkle] = M_PI/2.;
    q[dracobip_joint::rAnkle] = M_PI/2.;

    qdot[dracobip_joint::virtual_Ry] = 1.0;
    qdot[dracobip_joint::lAnkle] = 1.0;
    dynacore::Matrix J;
    dynacore::Vector JdotQdot;
    dynacore::Vect3 pos, vel, ang_vel;
     // Orientation
    dynacore::Vect3 rpy; rpy.setZero();
    dynacore::Quaternion quat_floating, link_ori_quat;
    rpy[1] = M_PI/4.;
    dynacore::convert(rpy, quat_floating);

    q[dracobip_joint::virtual_Rx] = quat_floating.x();
    q[dracobip_joint::virtual_Ry] = quat_floating.y();
    q[dracobip_joint::virtual_Rz] = quat_floating.z();
    q[dracobip_joint::virtual_Rw] = quat_floating.w();

    robot_sys_->UpdateSystem(q, qdot);
    int link_idx = dracobip_link::lAnkle;

    robot_sys_->getFullJacobian(link_idx, J);
    robot_sys_->getFullJDotQdot(link_idx, JdotQdot);
    robot_sys_->getOri(link_idx, link_ori_quat);
    robot_sys_->getPos(link_idx, pos);
    robot_sys_->getLinearVel(link_idx, vel);
    robot_sys_->getAngularVel(link_idx, ang_vel);
    
    Eigen::Matrix3d ori_rot(link_ori_quat); 
    dynacore::Matrix ori_rot_mt(6,6); ori_rot_mt.setZero();
    ori_rot_mt.topLeftCorner(3,3) = ori_rot;
    ori_rot_mt.bottomRightCorner(3,3) = ori_rot;

    dynacore::Matrix J_rot = ori_rot_mt.transpose() * J;

    dynacore::pretty_print(q, std::cout, "q");
    dynacore::pretty_print(qdot, std::cout, "qdot");
    dynacore::pretty_print(J, std::cout, "Jacobian");
    dynacore::pretty_print(J_rot, std::cout, "Jacobian Local");
    dynacore::pretty_print(link_ori_quat, std::cout, "ori");
    dynacore::pretty_print(JdotQdot, std::cout, "JdotQdot");
    dynacore::pretty_print(pos, std::cout, "pos");
    dynacore::pretty_print(vel, std::cout, "vel");
    dynacore::pretty_print(ang_vel, std::cout, "ang_vel");
    exit(0);
}

