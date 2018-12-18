#include "Quadruped_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include "common/utils.h"

//#include <DynaController/Quadruped_Controller/Quadruped_interface.hpp>
//#include <DynaController/Quadruped_Controller/Quadruped_DynaCtrl_Definition.h>
//#include <RobotSystems/Quadruped/Quadruped_Definition.h>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>

Quadruped_Dyn_environment::Quadruped_Dyn_environment():
    ang_vel_(3)
{
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());
    /********** Robot Set  **********/
    robot_ = new Quadruped();
    robot_->BuildRobot(Vec3 (0., 0., 0.), 
            srSystem::FIXED, srJoint::TORQUE, ModelPath"Quadruped/quadruped.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    //interface_ = new Quadruped_interface();
    //data_ = new Quadruped_SensorData();
    //cmd_ = new Quadruped_Command();


    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(0.0001);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(20);
    printf("[Quadruped Dynamic Environment] Build Dynamic Environment\n");
}

void Quadruped_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    Quadruped_Dyn_environment* pDyn_env = (Quadruped_Dyn_environment*)_data;
    Quadruped* robot = (Quadruped*)(pDyn_env->robot_);
    //Quadruped_SensorData* p_data = pDyn_env->data_;
    
    //std::vector<double> torque_command(robot->num_act_joint_);

    //for(int i(0); i<robot->num_act_joint_; ++i){
        //p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        //p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        //p_data->jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    //}
    //pDyn_env->_CheckFootContact(
            //p_data->rfoot_contact, p_data->lfoot_contact);
    //std::vector<double> imu_acc(3);
    //std::vector<double> imu_ang_vel(3);
    //pDyn_env->getIMU_Data(imu_acc, imu_ang_vel);
    //for (int i(0); i<3; ++i){
        //p_data->imu_ang_vel[i] = imu_ang_vel[i];
        //p_data->imu_acc[i] = -imu_acc[i];
    //}
    //pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    // Set Command
    //for(int i(0); i<3; ++i){
        //robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        //robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    //}

    //if( count < 100 ){
        //robot->vp_joint_[0]->m_State.m_rCommand = 
            //-1000. * robot->vp_joint_[0]->m_State.m_rValue[0]
            //- 10. * robot->vp_joint_[0]->m_State.m_rValue[1];
        //robot->vp_joint_[1]->m_State.m_rCommand = 
            //-1000. * robot->vp_joint_[1]->m_State.m_rValue[0]
            //- 10. * robot->vp_joint_[1]->m_State.m_rValue[1];
    //}

    //double Kp(200.);
    //double Kd(5.);
    //double ramp(1.);
    //if( count < 10 ){
        //ramp = ((double)count)/10.;
    //}
    //for(int i(0); i<robot->num_act_joint_; ++i){
        //robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            //Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            //Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
    //}
}


void Quadruped_Dyn_environment::Rendering_Fnc(){}
void Quadruped_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
    SO3 so3_body =  robot_->
        link_[robot_->link_idx_map_.find("body")->second]->GetOrientation();

    Eigen::Matrix3d ori_mtx;
    for (int i(0); i<3; ++i){
        ori_mtx(i, 0) = so3_body[0+i];
        ori_mtx(i, 1) = so3_body[3+i];
        ori_mtx(i, 2) = so3_body[6+i];
    }
    dynacore::Quaternion ori_quat(ori_mtx);
    rot = ori_quat;
}
Quadruped_Dyn_environment::~Quadruped_Dyn_environment()
{
    //SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}

void Quadruped_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("l_foot")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("r_foot")->second]->GetPosition();

    //std::cout<<rfoot_pos<<std::endl;
    //std::cout<<lfoot_pos<<std::endl;

    if(  fabs(lfoot_pos[2]) < 0.016){
        l_contact = true;
        //printf("left contact\n");
    }else { l_contact = false; }
    if (fabs(rfoot_pos[2])<0.016  ){
        r_contact = true;
        //printf("right contact\n");
    } else { r_contact = false; }

    //printf("\n");
}
void Quadruped_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = 
      robot_->link_[robot_->link_idx_map_.find("body")->second]->GetVel();
  se3 imu_se3_acc = 
      robot_->link_[robot_->link_idx_map_.find("body")->second]->GetAcc();
  SE3 imu_frame = 
      robot_->link_[robot_->link_idx_map_.find("body")->second]->GetFrame();
  SO3 imu_ori = 
      robot_->link_[robot_->link_idx_map_.find("body")->second]->GetOrientation();

  Eigen::Matrix3d Rot;
  Rot<<
    imu_frame(0,0), imu_frame(0,1), imu_frame(0,2),
    imu_frame(1,0), imu_frame(1,1), imu_frame(1,2),
    imu_frame(2,0), imu_frame(2,1), imu_frame(2,2);

  dynacore::Vect3 grav; grav.setZero();
  grav[2] = 9.81;
  dynacore::Vect3 local_grav = Rot.transpose() * grav;

  for(int i(0); i<3; ++i){
    imu_ang_vel[i] = imu_se3_vel[i];
    imu_acc[i] = local_grav[i];
  }
}
