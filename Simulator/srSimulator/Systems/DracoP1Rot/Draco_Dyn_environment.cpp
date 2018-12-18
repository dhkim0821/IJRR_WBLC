#include "draco.h"
#include "Draco_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>
#include "common/utils.h"

#ifdef Measure_Time
#include <chrono>
using namespace std::chrono;
#endif

#include <srConfiguration.h>
#include <Configuration.h>
#include <srTerrain/Ground.h>
#include <srTerrain/Config_Space_Definition.h>
#include <DracoP1Rot_Controller/StateProvider.hpp>
#include <DracoP1Rot_Controller/interface.hpp>

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay
#define ENVIRONMENT_SETUP 0

Draco_Dyn_environment::Draco_Dyn_environment():
  indicated_contact_pt_list_(4),
  commanded_contact_force_list_(4),
  ang_vel_(3)
{

  /********** Space Setup **********/
  m_Space = new srSpace();
  m_ground = new Ground();
  m_Space->AddSystem(m_ground->BuildGround());
  robot_ = new srDraco(Vec3(0.0, 0., 0.));
  m_Space->AddSystem((srSystem*)robot_);

  interface_ = new Interface();
  // contact_pt_list_.clear();
  // contact_force_list_.clear();

  m_Space->DYN_MODE_PRESTEP();
  m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
  m_Space->SetTimestep(SERVO_RATE);
  m_Space->SetGravity(0.0,0.0,-9.81);

  m_Space->SetNumberofSubstepForRendering(15);

  printf("[Draco Dynamic Environment] Build Dynamic Environment\n");
}

void Draco_Dyn_environment::ContolFunction( void* _data ) {
  static int count(0);
  ++count;

  Draco_Dyn_environment* pDyn_env = (Draco_Dyn_environment*)_data;
  srDraco* robot = (srDraco*)(pDyn_env->robot_);
  double alternate_time = SIM_SERVO_RATE * count;
  std::vector<double> jpos(robot->num_act_joint_);
  std::vector<double> jvel(robot->num_act_joint_);
  std::vector<double> jtorque(robot->num_act_joint_);
  std::vector<double> pos(2);
  std::vector<double> body_vel(2);
  std::vector<double> torque_command(robot->num_act_joint_, 0.);

  for(int i(0); i<robot->num_act_joint_; ++i){
    jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
    jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
  }
  double ori = robot->vr_joint_[0]->m_State.m_rValue[0];
  double ang_vel = robot->vr_joint_[0]->m_State.m_rValue[1];

  for (int i(0); i<2; ++i){
    pos[i] = robot->vp_joint_[i]->m_State.m_rValue[0];
    body_vel[i] = robot->vp_joint_[i]->m_State.m_rValue[1];
  }
  pDyn_env->interface_->GetCommand(alternate_time, jpos, jvel, jtorque, pos, body_vel, ori, ang_vel, torque_command);

  for(int i(0); i<2; ++i){
    robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
  }

  for(int i(0); i<robot->num_act_joint_; ++i){
    robot->r_joint_[i]->m_State.m_rCommand = torque_command[i];
  }
  // Push back
  // pDyn_env->_PushBody(count, -90., 0., 0., 2000, 2700); // Fx, Fy, Fz, start, end
  // pDyn_env->_PushBody(count, 10., 0., 0., 1000, 1300); // Fx, Fy, Fz, start, end
  // Push down
  // pDyn_env->_PushBody(count, 0., 0., -70., 1000, 1300); // Fx, Fy, Fz, start, end

}

void Draco_Dyn_environment::_PushBody(const int & curr_count,
                                      double Fx, double Fy, double Fz,
                                      int start_count, int end_count){
  if (start_count < curr_count && curr_count< end_count){
    dse3 ext_force(0., 0., 0., Fx, Fy, Fz);
    robot_->link_[robot_->link_idx_map_.find("body")->second]->AddUserExternalForce(ext_force);
  }
}


void Draco_Dyn_environment::Rendering_Fnc()
{
  // _Draw_Contact_Point();
  // _Draw_Contact_Force();
  // _Draw_Commanded_Force();
  // _Draw_Path();
  // _Draw_FootPlacement();
}
void Draco_Dyn_environment::_Draw_Contact_Point(){
  double radi(0.02);

  double theta(0.0);
  sejong::Vect3 contact_loc;
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
void Draco_Dyn_environment::_ListCommandedReactionForce(const sejong::Vector & Fr){
  sejong::Vect3 vec3_fr;
  sejong::Vect3 vec3_cp;
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

void Draco_Dyn_environment::_Draw_Commanded_Force(){
  sejong::Vect3 contact_loc;
  sejong::Vect3 contact_force;
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

void Draco_Dyn_environment::_Save_Orientation_Matrix(){
  SO3 so3_body =  robot_->link_[0]->GetOrientation();

  for (int i(0); i<3; ++i){
    ori_mtx_[0 + 3*i] = so3_body[0+i];
    ori_mtx_[1 + 3*i] = so3_body[3+i];
    ori_mtx_[2 + 3*i] = so3_body[6+i];
  }
}

void Draco_Dyn_environment::_Get_Orientation(sejong::Quaternion & rot){
  SO3 so3_body =  robot_->link_[robot_->link_idx_map_.find("body")->second]->GetOrientation();

  Eigen::Matrix3d ori_mtx;
  for (int i(0); i<3; ++i){
    ori_mtx(i, 0) = so3_body[0+i];
    ori_mtx(i, 1) = so3_body[3+i];
    ori_mtx(i, 2) = so3_body[6+i];
  }
  sejong::Quaternion ori_quat(ori_mtx);
  rot = ori_quat;
}

Draco_Dyn_environment::~Draco_Dyn_environment()
{
  //SR_SAFE_DELETE(interface_);
  SR_SAFE_DELETE(robot_);
  SR_SAFE_DELETE(m_Space);
  SR_SAFE_DELETE(m_ground);
}
