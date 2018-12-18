#include "Cassie_Dyn_environment.hpp"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include "common/utils.h"

//#include <DynaController/Cassie_Controller/Cassie_interface.hpp>
//#include <DynaController/Cassie_Controller/Cassie_DynaCtrl_Definition.h>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>
#include <ParamHandler/ParamHandler.hpp>

Cassie_Dyn_environment::Cassie_Dyn_environment():
    ang_vel_(3)
{
    _ParamterSetup();
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());
    /********** Robot Set  **********/
    robot_ = new Cassie();
    robot_->BuildRobot(Vec3 (0., 0., 0.), 
            srSystem::FIXED, srJoint::TORQUE, ModelPath"Cassie/Cassie.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    //interface_ = new Cassie_interface();
    //data_ = new Cassie_SensorData();
    //cmd_ = new Cassie_Command();


    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(1./1500.);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(num_substep_rendering_);

    printf("[Cassie Dynamic Environment] Build Dynamic Environment\n");
}

void Cassie_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    Cassie_Dyn_environment* pDyn_env = (Cassie_Dyn_environment*)_data;
    Cassie* robot = (Cassie*)(pDyn_env->robot_);
    Cassie_SensorData* p_data = pDyn_env->data_;

    std::vector<double> torque_command(robot->num_act_joint_);

    //for(int i(0); i<robot->num_act_joint_; ++i){
        //p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        //p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        //p_data->torque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    //}
    //pDyn_env->_CheckFootContact(p_data->rfoot_contact, p_data->lfoot_contact);
    //for (int i(0); i<3; ++i){
        //p_data->imu_ang_vel[i] = 
            //robot->link_[robot->link_idx_map_.find("torso")->second]->GetVel()[i];
    //}
    //pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    //pDyn_env->_ZeroInput_VirtualJoint();
    //pDyn_env->_hold_XY(count);

    //double Kp(10.);
    //double Kd(1.);
    //for(int i(0); i<robot->num_act_joint_; ++i){
        //robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            //Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            //Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
    //}
}


void Cassie_Dyn_environment::Rendering_Fnc(){
}
void Cassie_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
    SO3 so3_body =  robot_->
        link_[robot_->link_idx_map_.find("pelvis")->second]->GetOrientation();

    Eigen::Matrix3d ori_mtx;
    for (int i(0); i<3; ++i){
        ori_mtx(i, 0) = so3_body[0+i];
        ori_mtx(i, 1) = so3_body[3+i];
        ori_mtx(i, 2) = so3_body[6+i];
    }
    dynacore::Quaternion ori_quat(ori_mtx);
    rot = ori_quat;
}
Cassie_Dyn_environment::~Cassie_Dyn_environment()
{
    //SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}

void Cassie_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("lAnkle")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("rAnkle")->second]->GetPosition();

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

void Cassie_Dyn_environment::_hold_XY(int count){
    //static double initial_x(0.); 
    //if(count == 1){
        //initial_x = robot_->vp_joint_[0]->m_State.m_rValue[0];
    //}
    //if( ((double)count*sagitP3::servo_rate) < release_time_ ){
        //robot_->vp_joint_[0]->m_State.m_rCommand = 
            //10000. * (initial_x - robot_->vp_joint_[0]->m_State.m_rValue[0])
            //- 100. * robot_->vp_joint_[0]->m_State.m_rValue[1];
        //robot_->vp_joint_[1]->m_State.m_rCommand = 
            //-10000. * robot_->vp_joint_[1]->m_State.m_rValue[0]
            //- 100. * robot_->vp_joint_[1]->m_State.m_rValue[1];
    //}
}

void Cassie_Dyn_environment::_ZeroInput_VirtualJoint(){
    for(int i(0); i<3; ++i){
        robot_->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot_->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }
}

void Cassie_Dyn_environment::_ParamterSetup(){
  //ParamHandler handler(CassieConfigPath"SIM_sr_sim_setting.yaml");
  //handler.getInteger("num_substep_rendering", num_substep_rendering_);
  //handler.getValue("releasing_time", release_time_);
  //handler.getVector("imu_angular_velocity_bias", imu_ang_vel_bias_);
  //handler.getVector("imu_angular_velocity_noise_variance", imu_ang_vel_var_);
}


