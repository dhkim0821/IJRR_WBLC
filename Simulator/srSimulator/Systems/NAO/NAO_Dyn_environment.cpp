#include "NAO_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>
#include "common/utils.h"

#include <srConfiguration.h>
#include <Configuration.h>
#include <srTerrain/Ground.h>
#include <srTerrain/Config_Space_Definition.h>

#include <DynaController/NAO_Controller/NAO_interface.hpp>
#include <RobotSystems/NAO/NAO_Definition.h>


NAO_Dyn_environment::NAO_Dyn_environment():
    indicated_contact_pt_list_(4),
    commanded_contact_force_list_(4),
    ang_vel_(3)
{

    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();
    m_Space->AddSystem(m_ground->BuildGround());

    robot_ = new srNao();
    robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            srJoint::TORQUE, ModelPath"NAO_Model/urdf/nao_simple.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    interface_ = new NAO_interface();
    //contact_pt_list_.clear();
    contact_force_list_.clear();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
    m_Space->SetTimestep(0.001);
    m_Space->SetGravity(0.0,0.0,-9.81);
    m_Space->SetNumberofSubstepForRendering(20);

    printf("[Nao Dynamic Environment] Build Dynamic Environment\n");
}

void NAO_Dyn_environment::ContolFunction( void* _data ) {
    static int count(0);
    ++count;

    NAO_Dyn_environment* pDyn_env = (NAO_Dyn_environment*)_data;
    srNao* robot = (srNao*)(pDyn_env->robot_);
    NAO_SensorData* p_data = pDyn_env->data_;

    std::vector<double> torque_command(robot->num_act_joint_);

    for(int i(0); i<robot->num_act_joint_; ++i){
        p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        p_data->jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    }
    pDyn_env->_CheckFootContact(
            p_data->rfoot_contact, p_data->lfoot_contact);
    
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    pDyn_env->getIMU_Data(imu_acc, imu_ang_vel);
    for (int i(0); i<3; ++i){
        p_data->imu_ang_vel[i] = imu_ang_vel[i];
        p_data->imu_acc[i] = -imu_acc[i];
    }
    pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    for(int i(0); i<3; ++i){
        robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }

    double Kp(50.);
    double Kd(5.);
     //double Kp(30.);
     //double Kd(0.5);
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
    }
}

void NAO_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("l_ankle")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("r_ankle")->second]->GetPosition();

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
void NAO_Dyn_environment::Rendering_Fnc()
{
    //_Draw_CoM3DAcc_Point();
    //_Draw_CoMAcc();

    //_Draw_Contact_Point();
    // _Draw_Contact_Force();
    //_Draw_Commanded_Force();
    // _Draw_Path();
    // _Draw_FootPlacement();
}

void NAO_Dyn_environment::_Draw_Contact_Point(){
    double radi(0.02);

    double theta(0.0);
    dynacore::Vect3 contact_loc;
    for (int j(0); j<contact_pt_list_.size(); ++j){
        contact_loc = contact_pt_list_[j];
        glBegin(GL_LINE_LOOP);
        for (int i(0); i<3600; ++i){
            theta += DEG2RAD(i*0.1);
            glColor4f(0.5f, 0.1f, 0.5f, 0.1f);

            glVertex3f(contact_loc[0] + cos(theta)*radi, contact_loc[1] + sin(theta)*radi, contact_loc[2]);
        }
        glEnd();
    }
}
void NAO_Dyn_environment::_ListCommandedReactionForce(const dynacore::Vector & Fr){
    dynacore::Vect3 vec3_fr;
    dynacore::Vect3 vec3_cp;
    Vec3 contact_point;

    for(int i(0); i<4; ++i){
        contact_point = robot_->link_[robot_->link_idx_map_.find("FootOutFront")->second+ i]->GetPosition();
        for (int j(0); j<3; ++j){
            vec3_cp[j] = contact_point[j];
            vec3_fr[j] = Fr[3*i + j];
        }
        indicated_contact_pt_list_[i] = vec3_cp;
        commanded_contact_force_list_[i] = vec3_fr;
    }
}

void NAO_Dyn_environment::_Draw_Commanded_Force(){
    dynacore::Vect3 contact_loc;
    dynacore::Vect3 contact_force;
    double reduce_ratio(0.001);

    for (int j(0); j<indicated_contact_pt_list_.size(); ++j){
        glLineWidth(2.5);
        glColor4f(0.9f, 0.0f, 0.0f, 0.8f);
        glBegin(GL_LINES);

        contact_loc = indicated_contact_pt_list_[j];
        contact_force = reduce_ratio * commanded_contact_force_list_[j];
        contact_force += contact_loc;

        glVertex3f(contact_loc[0],  contact_loc[1],  contact_loc[2]);
        glVertex3f(contact_force[0], contact_force[1], contact_force[2]);
        glEnd();
    }
}
void NAO_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = 
      robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetVel();
  se3 imu_se3_acc = 
      robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetAcc();
  SE3 imu_frame = 
      robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetFrame();
  SO3 imu_ori = 
      robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetOrientation();

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

void NAO_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
    SO3 so3_body =  robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetOrientation();

    Eigen::Matrix3d ori_mtx;
    for (int i(0); i<3; ++i){
        ori_mtx(i, 0) = so3_body[0+i];
        ori_mtx(i, 1) = so3_body[3+i];
        ori_mtx(i, 2) = so3_body[6+i];
    }
    dynacore::Quaternion ori_quat(ori_mtx);
    rot = ori_quat;
}

NAO_Dyn_environment::~NAO_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}


