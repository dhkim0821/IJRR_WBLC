#include "DracoBip_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include <Utils/utilities.hpp>
#include "common/utils.h"

#include <DynaController/DracoBip_Controller/DracoBip_interface.hpp>
#include <DynaController/DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utils/Clock.hpp>

DracoBip_Dyn_environment::DracoBip_Dyn_environment():
    ang_vel_(3), simulation_freq_(10.)
{
    _ParamterSetup();
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());
    /********** Robot Set  **********/
    robot_ = new DracoBip();
    robot_->BuildRobot(Vec3 (0., 0., 0.), 
            srSystem::FIXED, srJoint::TORQUE, ModelPath"DracoBip/DracoBip.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    interface_ = new DracoBip_interface();
    data_ = new DracoBip_SensorData();
    cmd_ = new DracoBip_Command();


    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(dracobip::servo_rate/simulation_freq_);
    //m_Space->SetTimestep(1./15000.);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(num_substep_rendering_);

    printf("[DracoBip Dynamic Environment] Build Dynamic Environment\n");
}

void DracoBip_Dyn_environment::ControlFunction( void* _data ) {

    DracoBip_Dyn_environment* pDyn_env = (DracoBip_Dyn_environment*)_data;
    DracoBip* robot = (DracoBip*)(pDyn_env->robot_);
    DracoBip_SensorData* p_data = pDyn_env->data_;
    pDyn_env->count_++;
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
    dynacore::Clock clock;
    if (pDyn_env->count_%((int)pDyn_env->simulation_freq_) ==1){
        clock.start();
     pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 
     //if(pDyn_env->count_% 100 == 1) printf("clock:%f\n", clock.stop());
    }

    pDyn_env->_ZeroInput_VirtualJoint();
    pDyn_env->_hold_XY();
    pDyn_env->_hold_Orientation();

    double Kp(150.);
    double Kd(15.);
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
    }
    //if( ( (double)(pDyn_env->count_)*dracobip::servo_rate ) 
            //> pDyn_env->release_time_ ){
        //robot->r_joint_[4]->m_State.m_rCommand = 0.; //pDyn_env->cmd_->jtorque_cmd[4]; 
        //robot->r_joint_[9]->m_State.m_rCommand = 0.; // pDyn_env->cmd_->jtorque_cmd[9];
    //}
  //pDyn_env->PushRobotBody();
}

void DracoBip_Dyn_environment::Rendering_Fnc(){  }
void DracoBip_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
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
DracoBip_Dyn_environment::~DracoBip_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}
void DracoBip_Dyn_environment::PushRobotBody(){
  static int push_idx(0);
  if(push_time_.size() > push_idx){

    if(count_ * dracobip::servo_rate/ simulation_freq_> push_time_[push_idx]){
      int body_lk_idx(robot_->link_idx_map_.find("torso")->second);
      // Force Setting
      double force_size(push_force_[push_idx]);
      double force_dir(DEG2RAD(push_direction_[push_idx]));

      double force_x = -force_size * cos(force_dir);
      double force_y = -force_size * sin(force_dir);

      dse3 ext_force(0., 0., 0.,force_x, force_y, 0.);
      robot_->link_[body_lk_idx]->AddUserExternalForce(ext_force);

      if(count_*dracobip::servo_rate/simulation_freq_ > push_time_[push_idx] + 0.1){
        ++push_idx;
      }
    }
  }
}
void DracoBip_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
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

void DracoBip_Dyn_environment::_hold_XY(){
    static double initial_x(0.); 
    if(count_ == 1){
        initial_x = robot_->vp_joint_[0]->m_State.m_rValue[0];
    }
    if( ((double)count_*dracobip::servo_rate) < release_time_ ){
        robot_->vp_joint_[0]->m_State.m_rCommand = 
            1000. * (initial_x - robot_->vp_joint_[0]->m_State.m_rValue[0])
            - 50. * robot_->vp_joint_[0]->m_State.m_rValue[1];
        robot_->vp_joint_[1]->m_State.m_rCommand = 
            -1000. * robot_->vp_joint_[1]->m_State.m_rValue[0]
            - 50. * robot_->vp_joint_[1]->m_State.m_rValue[1];
    }
}

void DracoBip_Dyn_environment::_hold_Orientation(){
    static double initial_x(0.); 
    double pos,vel;

    double kp(500.0);
    double kd(15);

    if( ((double)count_*dracobip::servo_rate) < release_time_ ){
        int idx(1);
        pos = robot_->vr_joint_[idx]->m_State.m_rValue[0];
        vel = robot_->vr_joint_[idx]->m_State.m_rValue[1];
        robot_->vr_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

        idx = 2;
        pos = robot_->vr_joint_[idx]->m_State.m_rValue[0];
        vel = robot_->vr_joint_[idx]->m_State.m_rValue[1];

        idx = 0;
        pos = robot_->vr_joint_[idx]->m_State.m_rValue[0];
        vel = robot_->vr_joint_[idx]->m_State.m_rValue[1];


        robot_->vr_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;
    }
}

void DracoBip_Dyn_environment::_ZeroInput_VirtualJoint(){
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

void DracoBip_Dyn_environment::_ParamterSetup(){
  ParamHandler handler(DracoBipConfigPath"SIM_sr_sim_setting.yaml");
  handler.getInteger("num_substep_rendering", num_substep_rendering_);
  handler.getValue("releasing_time", release_time_);
  handler.getVector("imu_angular_velocity_bias", imu_ang_vel_bias_);
  handler.getVector("imu_angular_velocity_noise_variance", imu_ang_vel_var_);

  handler.getVector("push_timing", push_time_);
  handler.getVector("push_force", push_force_);
  handler.getVector("push_direction", push_direction_);
}

void DracoBip_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetVel();
  se3 imu_se3_acc = robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetAcc();
  SE3 imu_frame = robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetFrame();
  SO3 imu_ori = robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetOrientation();

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
