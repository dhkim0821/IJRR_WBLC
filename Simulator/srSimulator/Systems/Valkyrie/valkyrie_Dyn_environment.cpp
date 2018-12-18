#include "valkyrie_Dyn_environment.h"
#include <iostream>
#include <stdio.h>
#include "srDyn/srSpace.h"
#include "common/utils.h"

#include <DynaController/Valkyrie_Controller/Valkyrie_interface.hpp>
#include <DynaController/Valkyrie_Controller/Valkyrie_DynaCtrl_Definition.h>
#include <DynaController/Valkyrie_Controller/Valkyrie_StateProvider.hpp>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>

Valkyrie_Dyn_environment::Valkyrie_Dyn_environment()
{
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());

    /********** Robot Set  **********/
    robot_ = new New_Valkyrie();
    //robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
    //srJoint::TORQUE, ModelPath"Valkyrie_Model/r5_urdf.urdf");
    robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            srJoint::TORQUE, ModelPath"Valkyrie/valkyrie_simple.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    interface_ = new Valkyrie_interface();
    data_ = new Valkyrie_SensorData();
    cmd_ = new Valkyrie_Command();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(valkyrie::servo_rate);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(50);

  sp_ = Valkyrie_StateProvider::getStateProvider();
    printf("[Valkyrie Dynamic Environment] Build Dynamic Environment\n");
}

void Valkyrie_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    Valkyrie_Dyn_environment* pDyn_env = (Valkyrie_Dyn_environment*)_data;
    New_Valkyrie* robot = (New_Valkyrie*)(pDyn_env->robot_);
    Valkyrie_SensorData* p_data = pDyn_env->data_;

    //printf("num act joint: %d \n", robot->num_act_joint_);
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

    // Set Command
    for(int i(0); i<3; ++i){
        robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }

    if( count < 100 ){
        robot->vp_joint_[0]->m_State.m_rCommand = 
            -15000. * robot->vp_joint_[0]->m_State.m_rValue[0]
            - 500. * robot->vp_joint_[0]->m_State.m_rValue[1];
        robot->vp_joint_[1]->m_State.m_rCommand = 
            -15000. * robot->vp_joint_[1]->m_State.m_rValue[0]
            - 500. * robot->vp_joint_[1]->m_State.m_rValue[1];
    }

    double Kp(100.);
    double Kd(5.);
     //double Kp(30.);
     //double Kd(0.5);
    // Right
    double ramp(1.);
    if( count < 10 ){
        ramp = ((double)count)/10.;
    }
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
        
        //robot->r_joint_[i]->m_State.m_rCommand *= ramp;
    }
}

void Valkyrie_Dyn_environment::Rendering_Fnc()
{
    _DrawDesiredLocation();
}

void Valkyrie_Dyn_environment::_DrawDesiredLocation(){
  // Attraction Location
  double radi(0.07);
  double theta(0.0);
   //double height_attraction_loc = 
   //Terrain_Interpreter::GetTerrainInterpreter()->surface_height( HUME_System::GetHumeSystem()->state_provider_->attraction_loc_[0],
  //                                                                                              HUME_System::GetHumeSystem()->state_provider_->attraction_loc_[1]);

   double height_attraction_loc = 0.005;
   glBegin(GL_LINE_LOOP); 
   theta = 0.0; 
   for (int i(0); i<3600; ++i){ 
       theta += DEG2RAD(i*0.1);
       glColor3f(0.5f, 0.1f, 0.f); 
       glVertex3f(sp_->des_location_[0] + cos(theta)*radi,
                  sp_->des_location_[1] + sin(theta)*radi, height_attraction_loc );
   } 
   glEnd(); 
}


void Valkyrie_Dyn_environment::_DrawHollowCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius){
    int i;
    int lineAmount = 200; //# of triangles used to draw circle

    //GLfloat radius = 0.8f; //radius
    GLfloat twicePi = 2.0f * M_PI;

    for (int i(0); i< lineAmount-1; ++i){
        glLineWidth(2.0);
        glBegin(GL_LINES);
        glVertex3f(x + radius * cos(i * twicePi / lineAmount),
                y + radius * sin(i * twicePi / lineAmount), z);
        glVertex3f(x + radius * cos((i+1) * twicePi / lineAmount),
                y + radius * sin((i+1) * twicePi / lineAmount), z);
        glEnd();
    }
}

void Valkyrie_Dyn_environment::_Copy_Array(double * subject, double * data, int num_element){
    // for(int i(0); i< num_element; ++i){
    //     subject[i] = data[i];
    // }
}

Valkyrie_Dyn_environment::~Valkyrie_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}

void Valkyrie_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("leftCOP_Frame")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("rightCOP_Frame")->second]->GetPosition();

    //std::cout<<rfoot_pos<<std::endl;
    //std::cout<<lfoot_pos<<std::endl;

    if(  fabs(lfoot_pos[2]) < 0.015){
        l_contact = true;
        //printf("left contact\n");
    }else { l_contact = false; }
    if (fabs(rfoot_pos[2])<0.015  ){
        r_contact = true;
        //printf("right contact\n");
    } else { r_contact = false; }

    //printf("\n");
}

void Valkyrie_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = 
      robot_->link_[robot_->link_idx_map_.find("pelvis")->second]->GetVel();
  se3 imu_se3_acc = 
      robot_->link_[robot_->link_idx_map_.find("pelvis")->second]->GetAcc();
  SE3 imu_frame = 
      robot_->link_[robot_->link_idx_map_.find("pelvis")->second]->GetFrame();
  SO3 imu_ori = 
      robot_->link_[robot_->link_idx_map_.find("pelvis")->second]->GetOrientation();

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
