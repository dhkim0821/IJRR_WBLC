#include "SagitP3_Dyn_environment.hpp"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include "common/utils.h"

#include <DynaController/SagitP3_Controller/SagitP3_interface.hpp>
#include <DynaController/SagitP3_Controller/SagitP3_state_interface.hpp>
#include <DynaController/SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>
#include <Utils/utilities.hpp>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>
#include <ParamHandler/ParamHandler.hpp>

SagitP3_Dyn_environment::SagitP3_Dyn_environment():
    ang_vel_(3)
{
    _ParamterSetup();
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());
    /********** Robot Set  **********/
    robot_ = new SagitP3();
    robot_->BuildRobot(Vec3 (0., 0., 0.), 
            //srSystem::FIXED, srJoint::TORQUE, ModelPath"SagitP3/SagitP3.urdf");
            srSystem::FIXED, srJoint::TORQUE, ModelPath"SagitP3/p3_model_srlib.urdf");
            //srSystem::FIXED, srJoint::TORQUE, ModelPath"SagitP3/p3_model_coord.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    //interface_ = new SagitP3_interface();
    state_interface_ = new SagitP3_state_interface();
    data_ = new SagitP3_SensorData();
    state_data_ = new SagitP3_StateData();
    cmd_ = new SagitP3_Command();


    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(sagitP3::servo_rate);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(num_substep_rendering_);

    printf("[SagitP3 Dynamic Environment] Build Dynamic Environment\n");
}

void SagitP3_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    SagitP3_Dyn_environment* pDyn_env = (SagitP3_Dyn_environment*)_data;
    SagitP3* robot = (SagitP3*)(pDyn_env->robot_);
    SagitP3_SensorData* p_data = pDyn_env->data_;
    SagitP3_StateData* p_state_data = pDyn_env->state_data_;

    std::vector<double> torque_command(robot->num_act_joint_);

    for(int i(0); i<robot->num_act_joint_; ++i){
        p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        p_data->torque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    }
    
    pDyn_env->_CheckFootContact(p_data->rfoot_contact, p_data->lfoot_contact);

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    pDyn_env->getIMU_Data(imu_acc, imu_ang_vel);
    for (int i(0); i<3; ++i){
        p_data->imu_ang_vel[i] = imu_ang_vel[i];
        p_data->imu_acc[i] = imu_acc[i];
    }
    //pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    // State Interfacenterface
    dynacore::Quaternion ori_quat;
    dynacore::Vect3 ang_vel;
    pDyn_env->_Get_Orientation(ori_quat, ang_vel);
    //dynacore::pretty_print(ori_quat, std::cout, "ori quat");
    //dynacore::pretty_print(ang_vel, std::cout, "ang vel");
    for(int i(0); i<3; ++i){
        p_state_data->q[i] = robot->vp_joint_[i]->m_State.m_rValue[0];
        p_state_data->qdot[i] = robot->vp_joint_[i]->m_State.m_rValue[1];

        p_state_data->qdot[i+3] = ang_vel[i];
    } 
    p_state_data->q[sagitP3_joint::virtual_Rx] = ori_quat.x();
    p_state_data->q[sagitP3_joint::virtual_Ry] = ori_quat.y();
    p_state_data->q[sagitP3_joint::virtual_Rz] = ori_quat.z();
    p_state_data->q[sagitP3_joint::virtual_Rw] = ori_quat.w();

    for(int i(0); i<sagitP3::num_act_joint; ++i){
        p_state_data->q[i + sagitP3::num_virtual] = p_data->jpos[i];
        p_state_data->qdot[i + sagitP3::num_virtual] = p_data->jvel[i];
    }
    //dynacore::pretty_print(p_state_data->q, "q", sagitP3::num_q );
    //dynacore::pretty_print(p_state_data->qdot, "qdot", sagitP3::num_qdot );
    pDyn_env->state_interface_->GetCommand(p_state_data, pDyn_env->cmd_); 

    pDyn_env->_ZeroInput_VirtualJoint();
    pDyn_env->_hold_XY(count);
    //pDyn_env->_hold_Ori(count);

    double Kp(10.0);
    double Kd(1.0);
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
    }
}


void SagitP3_Dyn_environment::Rendering_Fnc(){
}
void SagitP3_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot, 
        dynacore::Vect3 & ang_vel){
    SO3 so3_body =  robot_->
        link_[robot_->link_idx_map_.find("hip_ground_link")->second]->GetOrientation();

    se3 body_vel =  robot_->
        link_[robot_->link_idx_map_.find("hip_ground_link")->second]->GetVel();


    Eigen::Matrix3d ori_mtx;
    for (int i(0); i<3; ++i){
        ori_mtx(i, 0) = so3_body[0+i];
        ori_mtx(i, 1) = so3_body[3+i];
        ori_mtx(i, 2) = so3_body[6+i];
        //
        ang_vel[i] = body_vel[i];
    }
    dynacore::Quaternion ori_quat(ori_mtx);
    rot = ori_quat;
}
SagitP3_Dyn_environment::~SagitP3_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}

void SagitP3_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("left_foot_link")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("right_foot_link")->second]->GetPosition();

    //std::cout<<rfoot_pos<<std::endl;
    //std::cout<<lfoot_pos<<std::endl;

    if(  fabs(lfoot_pos[2]) < 0.029){
        l_contact = true;
        //printf("left contact\n");
    }else { l_contact = false; }
    if (fabs(rfoot_pos[2])<0.029  ){
        r_contact = true;
        //printf("right contact\n");
    } else { r_contact = false; }

    //printf("\n");
}

void SagitP3_Dyn_environment::_hold_XY(int count){
    static double initial_x(0.); 
    if(count == 1){
        initial_x = robot_->vp_joint_[0]->m_State.m_rValue[0];
    }
    if( ((double)count*sagitP3::servo_rate) < release_time_ ){
        robot_->vp_joint_[0]->m_State.m_rCommand = 
            10000. * (initial_x - robot_->vp_joint_[0]->m_State.m_rValue[0])
            - 100. * robot_->vp_joint_[0]->m_State.m_rValue[1];
        robot_->vp_joint_[1]->m_State.m_rCommand = 
            -10000. * robot_->vp_joint_[1]->m_State.m_rValue[0]
            - 100. * robot_->vp_joint_[1]->m_State.m_rValue[1];
    }
}
void SagitP3_Dyn_environment::_hold_Ori(int count){
        robot_->vr_joint_[0]->m_State.m_rCommand = 
            10000. * ( 0. - robot_->vr_joint_[0]->m_State.m_rValue[0])
            - 100. * robot_->vr_joint_[0]->m_State.m_rValue[1];

        robot_->vr_joint_[1]->m_State.m_rCommand = 
            10000. * ( 0. - robot_->vr_joint_[1]->m_State.m_rValue[0])
            - 100. * robot_->vr_joint_[1]->m_State.m_rValue[1];

        robot_->vr_joint_[2]->m_State.m_rCommand = 
            10000. * ( SR_PI_HALF - robot_->vr_joint_[2]->m_State.m_rValue[0])
            - 100. * robot_->vr_joint_[2]->m_State.m_rValue[1];

}


void SagitP3_Dyn_environment::_ZeroInput_VirtualJoint(){
    for(int i(0); i<3; ++i){
        robot_->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot_->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }
}
//for(int i(0); i<robot->vr_joint_.size(); ++i){
//printf("%f\n",robot->vr_joint_[i]->m_State.m_rValue[0] );
//}
//for(int i(0); i<robot->vp_joint_.size(); ++i){
//printf("%f\n",robot->vp_joint_[i]->m_State.m_rValue[0] );
//}

//for(int i(0); i<robot->r_joint_.size(); ++i){
//printf("%f\n",robot->r_joint_[i]->m_State.m_rValue[0] );
//}
//printf("\n");
void SagitP3_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = robot_->link_[robot_->link_idx_map_.find("P3_IMU")->second]->GetVel();
  se3 imu_se3_acc = robot_->link_[robot_->link_idx_map_.find("P3_IMU")->second]->GetAcc();
  SE3 imu_frame = robot_->link_[robot_->link_idx_map_.find("P3_IMU")->second]->GetFrame();
  SO3 imu_ori = robot_->link_[robot_->link_idx_map_.find("P3_IMU")->second]->GetOrientation();

  Eigen::Matrix3d Rot;
  Rot<<
    imu_frame(0,0), imu_frame(0,1), imu_frame(0,2),
    imu_frame(1,0), imu_frame(1,1), imu_frame(1,2),
    imu_frame(2,0), imu_frame(2,1), imu_frame(2,2);

  dynacore::Vect3 grav; grav.setZero();
  grav[2] = 9.81;
  dynacore::Vect3 local_grav = Rot.transpose() * grav;

  for(int i(0); i<3; ++i){
    imu_ang_vel[i] = imu_se3_vel[i] + imu_ang_vel_bias_[i] +
      dynacore::generator_white_noise(0., imu_ang_vel_var_[i]);
    //imu_acc[i] = imu_se3_acc[i+3] - local_grav[i];
    imu_acc[i] = local_grav[i];
  }

  Eigen::Matrix<double, 3, 1> ang_vel;
  ang_vel<<imu_se3_vel[0], imu_se3_vel[1], imu_se3_vel[2];
  dynacore::Vect3 global_ang_vel = Rot * ang_vel;
  Eigen::Quaterniond quat(Rot);

  static int count = 0;

  //bool b_printout(true);
  bool b_printout(false);
  if (count % 100 == 0){
    if(b_printout){
      printf("imu info: \n");
      std::cout<<imu_se3_vel<<std::endl;
      std::cout<<imu_se3_acc<<std::endl;
      std::cout<<imu_frame<<std::endl;

      dynacore::pretty_print(imu_ang_vel, "imu ang vel");
      dynacore::pretty_print(imu_acc, "imu_acc");


      printf("global ang vel\n");
      std::cout<<global_ang_vel<<std::endl;

      printf("quat global:\n");
      std::cout<<quat.w()<<std::endl;
      std::cout<<quat.vec()<<std::endl;
    }
    count = 0;
  }
  count++;
}
void SagitP3_Dyn_environment::_ParamterSetup(){
  ParamHandler handler(SagitP3ConfigPath"SIM_sr_sim_setting.yaml");
  handler.getInteger("num_substep_rendering", num_substep_rendering_);
  handler.getValue("releasing_time", release_time_);
  handler.getVector("imu_angular_velocity_bias", imu_ang_vel_bias_);
  handler.getVector("imu_angular_velocity_noise_variance", imu_ang_vel_var_);
}

    //for(int i(0); i<robot->num_act_joint_; ++i){
        //printf("%d joint: %f\n", i, p_data->jpos[i]);
    //}
    //for(int i(0); i<3; ++i){
        //printf("virtual prismatic %d joint: %f\n", i, 
                //robot->vp_joint_[i]->m_State.m_rValue[0]);
        //printf("virtual rotary %d joint: %f\n", i, 
                //robot->vr_joint_[i]->m_State.m_rValue[0]);
    //}
    //printf("\n");

