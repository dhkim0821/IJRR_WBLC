#include "Dyn_environment.h" 
#include <iostream> 
#include <vector> 
 
//////////////////////////////////////////////// 
#ifdef __APPLE__ 
#include <GLUT/glut.h> 
#endif 
 
#ifdef linux 
#include <GL/glut.h> 
#endif 
//////////////////////////////////////////////// 
 
 
 
#include "common/utils.h" 
#include "utils/pseudo_inverse.hpp" 
#include "utils/wrap_eigen.hpp" 
#include <utils/utilities.h>

#include "Planner/terrain_interpreter.h"

#include <ControlSystem/Hume_Controller/Controller_Hume.h>
#include <ControlSystem/Hume_Controller/controller_Step.h> 
#include <ControlSystem/Hume_Controller/Process.h>

#include <ControlSystem/Hume_Controller/StateEstimator.h>
#include <ControlSystem/Hume_Controller/StateProvider.h>

#include <ControlSystem/Hume_Controller/interface.h>

#include "LED_Position_Announcer.h"
// #define Measure_Time

#ifdef Measure_Time
#include <chrono>
using namespace std::chrono;
#endif

std::vector<Vec3> g_trajectory; 
std::vector<Vec3> g_leftfoot; 
std::vector<Vec3> g_rightfoot; 
std::vector<Vec3> g_CoM_des_trajectory; 
extern simulation_setting * sim_setting; 

Hume_Interface interface_;

// #define SENSOR_NOISE 
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay 


Dyn_environment::Dyn_environment():
  curr_conf_(SIM_NUM_RJOINT), 
  curr_jvel_(SIM_NUM_RJOINT), 
  torque_   (SIM_NUM_RJOINT), 
  ang_vel_(3),
  acc_(3),
  left_foot_contact_(false), 
  right_foot_contact_(true) 
{ 
  m_Hume = new Hume();
  m_Hume->BuildRobot(Vec3(0.0, 0.0, 0.0), srSystem::FIXED, srJoint::TORQUE);
  m_Space = new srSpace(); 
  m_ground = new Ground(); 
  terrain_ = new Terrain();

  m_Space->AddSystem(m_ground->BuildGround()); 
  m_Space->AddSystem((srSystem*)m_Hume);
  m_Space->AddSystem((srSystem*)terrain_);
  m_Space->DYN_MODE_PRESTEP();


  m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction); 
  m_Space->SetTimestep(SERVO_RATE);
  m_Space->SetGravity(0.0,0.0,-9.8); 
  m_Space->SetNumberofSubstepForRendering(6); 
  // if (sim_setting->no_stabilizing_) 
  // { 
  //     ((stabilizing*)HUME_System::GetHumeSystem()->controller_->pro_stable_)->stable_move_time_ = 0.0005; 
  //     ((stabilizing*)HUME_System::GetHumeSystem()->controller_->pro_stable_)->stable_lifting_time_ = 0.0005; 
  //     ((stabilizing*)HUME_System::GetHumeSystem()->controller_->pro_stable_)->lifting_height_ = 0.1; 
  // } 

  landing_loc_ = Vector::Zero(2); 
     
  printf("left foot height: %f, \n",m_Hume->m_Link[HumeID::SIM_LFOOT].GetMassCenter()[2]); 
  printf("right foot height: %f, \n",m_Hume->m_Link[HumeID::SIM_RFOOT].GetMassCenter()[2]); 
 
  LED_pos_announcer_ = new LED_Position_Announcer(this);

  LED_pos_announcer_->start(); 
}	 

void Dyn_environment::ContolFunction( void* _data ) 
{ 
  static int iter(0); 

  Dyn_environment* pDyn_env = (Dyn_environment*)_data; 
  pDyn_env->SetCurrentState_All(); 

  int return_state; 
  std::vector<double> command; 
  if(iter<SENSOR_DELAY){ 
    return_state = iter; 
  } 
  else{ 
    return_state = iter - SENSOR_DELAY; 
  } 

  std::vector<int32_t> kp_torque(6); 
  std::vector<int32_t> ki_torque(6); 
  std::vector<double>  dummy_acc_mag(3);

  std::vector<int> dummy;
  std::vector<int> dac_command;

  _UpdateStateProvider(pDyn_env->m_Hume);
    
  interface_.GetCommand(
                        pDyn_env->history_state_[return_state].conf, 
                        pDyn_env->history_state_[return_state].jvel, 
                        pDyn_env->history_state_[return_state].torque,
                        pDyn_env->history_state_[return_state].torque,
                        dummy,
                        pDyn_env->history_state_[return_state].ori_mtx, 
                        pDyn_env->history_state_[return_state].ang_vel, 
                        pDyn_env->history_state_[return_state].accelerometer, 
                        dummy_acc_mag, 
                        pDyn_env->history_state_[return_state].left_foot_contact, 
                        pDyn_env->history_state_[return_state].right_foot_contact, 
                        kp_torque, 
                        ki_torque,
                        ki_torque,
                        command,
                        dac_command);
        
  for(int i(0); i<SIM_NUM_RJOINT; ++i){ 
    pDyn_env->m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rCommand =
      ((double)dac_command[i]*1.0)/FLOATING_PT
      + 150.0 * (command[i] - pDyn_env->m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rValue[0])
      + 10.0 *( - pDyn_env->m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rValue[1]); 
  }
        
  ////////////////////// 
  //  PASSIVE Joint   // 
  //////////////////////
  for (int i(0); i<SIM_NUM_PASSIVE; ++i){ 
    pDyn_env->m_Hume->Full_Joint_State_[i]->m_rCommand = 0.0; 
  } 
  // Torque Controller
  // _Torque_Controller(pDyn_env, kp_torque, ki_torque, command);

  if(interface_.GetHumeSystem()->controller_->phase_ == 10){
    _Hold_Rx(pDyn_env->m_Hume);
    _Hold_Ry(pDyn_env->m_Hume);
    _Hold_Rz(pDyn_env->m_Hume);
            
    _Hold_Y(pDyn_env->m_Hume);
    _Hold_X(pDyn_env->m_Hume);
  }

  ++iter;
}
void Dyn_environment::_UpdateStateProvider(Hume* hume){
  SO3 so3_body =  hume->m_Link[HumeID::SIM_HIP].GetOrientation();

  Eigen::Matrix3d ori_mtx;
    
  for (int i(0); i<3; ++i){
    for(int j(0); j<3; ++j){
      ori_mtx(j,i) = so3_body[3*i + j];
    }
  }
  sejong::Quaternion q_quat(ori_mtx);

  for(int i(0); i<NUM_QDOT; ++i){
    StateProvider::GetStateProvider()->Q_sim_[i] = hume->Full_Joint_State_[i]->m_rValue[0];
    StateProvider::GetStateProvider()->Qdot_sim_[i] = hume->Full_Joint_State_[i]->m_rValue[1];
    StateProvider::GetStateProvider()->Qddot_sim_[i] = hume->Full_Joint_State_[i]->m_rValue[2];
  }
  StateProvider::GetStateProvider()->Q_sim_[2] += hume->starting_height_;
  StateProvider::GetStateProvider()->Q_sim_[3] = q_quat.x();
  StateProvider::GetStateProvider()->Q_sim_[4] = q_quat.y();
  StateProvider::GetStateProvider()->Q_sim_[5] = q_quat.z();
  StateProvider::GetStateProvider()->Q_sim_[NUM_QDOT] = q_quat.w();


  StateProvider::GetStateProvider()->Qddot_sim_[0] = hume->m_Pjoint[0].m_State.m_rValue[2];
  for(int i(0); i<9; ++i){
    StateProvider::GetStateProvider()->Qddot_sim_[3 + i] = hume->m_Rjoint[i].m_State.m_rValue[2];
  }

  // printf("\n");
  // sejong::pretty_print(StateProvider::GetStateProvider()->Qdot_sim_, std::cout, "Qdot sim","");
  // sejong::pretty_print(StateProvider::GetStateProvider()->Qddot_sim_, std::cout, "Qddot sim","");
}

 
void Dyn_environment::saveLandingLocation(){ 
  // Ctrl_Step* ctrl_step = (Ctrl_Step*)HUME_System::GetHumeSystem()->controller_; 
  // double theta(0.0); 
  // Vec3 offset; 
  // if(ctrl_step->phase_ == LsRm || ctrl_step->phase_ == RsLm){ 
  //     // printf("landing_location:%f, %f \n", ctrl_step->landing_loc_[0], ctrl_step->landing_loc_[1]); 
  //     Vector land_loc(2); 
  //     // theta  = m_Hume->Full_Joint_State_[SIM_NUM_PASSIVE_P]->m_rValue[0]; 
  //     offset = m_Hume->m_Link[ctrl_step->base_pt_].GetMassCenter(); 
  //     land_loc[0] = ctrl_step->landing_loc_[0]*cos(theta) - ctrl_step->landing_loc_[1]*sin(theta); 
  //     land_loc[1] = ctrl_step->landing_loc_[0]*sin(theta) + ctrl_step->landing_loc_[1]*cos(theta); 
  //     land_loc[0] += offset[0]; 
  //     land_loc[1] += offset[1]; 
 
  //     landing_loc_ = land_loc; 
  // } 
} 
void Dyn_environment::_Save_Orientation_Matrix(){
  SO3 so3_body =  m_Hume->m_Link[HumeID::SIM_HIP].GetOrientation();
    
  for (int i(0); i<3; ++i){
    ori_mtx_[0 + 3*i] = so3_body[0+i];
    ori_mtx_[1 + 3*i] = so3_body[3+i];
    ori_mtx_[2 + 3*i] = so3_body[6+i];
  }

  for (int i(0); i<3; ++i){
    for(int j(0); j<3; ++j){
      sj_ori_mtx_(j,i) = so3_body[3*i + j];
    }
  }

  // std::cout<<"so3: "<<so3_body;
  // printf("body_ori in simulation: %f, %f, %f \n",
  //        m_Hume->Full_Joint_State_[3]->m_rValue[0],
  //        m_Hume->Full_Joint_State_[4]->m_rValue[0],           
  //        m_Hume->Full_Joint_State_[5]->m_rValue[0]);
  // printf("matrix simulation: \n %f, %f, %f \n  %f, %f, %f \n  %f, %f, %f \n",
  //        ori_mtx_[0], ori_mtx_[1], ori_mtx_[2],
  //        ori_mtx_[3], ori_mtx_[4], ori_mtx_[5],
  //        ori_mtx_[6], ori_mtx_[7], ori_mtx_[8]);
}
void Dyn_environment::SetCurrentState_All(){ 
 
  Vec3 acc_noise; 
  Vec3 ang_vel_noise; 
 
#ifdef SENSOR_NOISE 
  for (int i(0); i< 3; ++i){ 
    acc_noise[i] = 0.07*generator_white_noise(0.0, 1.0) + 0.01; 
    ang_vel_noise[i] = 0.17*generator_white_noise(0.0, 1.0); 
  } 
#endif 
  for (int i(0); i< SIM_NUM_RJOINT; ++i){ 
    curr_conf_[i] = m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rValue[0]; 
    curr_jvel_[i] = m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rValue[1]; 
    torque_ [i]   = m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rValue[3]; 
  }
  _Save_Orientation_Matrix();
  // R x, y, z 
  // for (int i(0); i<3; ++i){ 
  //     ang_vel_[2-i] = m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE_P] -> m_rValue[1] + ang_vel_noise[i]; 
  // }

  _Save_Orientation_Matrix();
  sejong::Vector sj_vertical(3);
  sj_vertical.setZero();
  sj_vertical[2] = 9.81;

  Vec3 l_acc;
  m_Hume->m_Link[HumeID::SIM_HIP].GetLinearAcc(l_acc);
  sejong::Vector sj_l_acc(3);
  sj_l_acc << l_acc[0], l_acc[1], l_acc[2];
  sj_l_acc += sj_vertical;
  sj_l_acc = sj_ori_mtx_.transpose() * sj_l_acc;

  sejong::Quaternion q(sj_ori_mtx_);
  // sejong::pretty_print(q, std::cout, "srlib quaternion","");
     
  for (int i(0); i<3; ++i){ 
    ang_vel_[i] = m_Hume->m_Link[HumeID::SIM_HIP].GetVel()[i] + ang_vel_noise[i];
    acc_[i] = sj_l_acc[i] + acc_noise[i];
  }
  // sejong::pretty_print(ang_vel_, "ang_velocity");

    
  if(m_Hume->m_Link[HumeID::SIM_LFOOT].GetMassCenter()[2] < SIM_FOOT_RADIUS + 0.005){ 
    left_foot_contact_ = true; 
  } 
  else{ left_foot_contact_ = false; } 
 
  if(m_Hume->m_Link[HumeID::SIM_RFOOT].GetMassCenter()[2] < SIM_FOOT_RADIUS + 0.005){ 
    right_foot_contact_ = true; 
  } 
  else{ right_foot_contact_ = false; } 
 
  state curr_state; 
  curr_state.conf               =   curr_conf_; 
  curr_state.jvel               =   curr_jvel_; 
  curr_state.torque             =   torque_   ;
  _Copy_Array(curr_state.ori_mtx, ori_mtx_, 9);
  curr_state.ang_vel            =   ang_vel_;
  curr_state.accelerometer      =   acc_;
  curr_state.left_foot_contact  =   left_foot_contact_; 
  curr_state.right_foot_contact =   right_foot_contact_; 
 
  history_state_.push_back(curr_state); 
} 
void Dyn_environment::_Copy_Array(double * subject, double * data, int num_element){
  for(int i(0); i< num_element; ++i){
    subject[i] = data[i];
  }
}
Dyn_environment::~Dyn_environment() 
{ 
  SR_SAFE_DELETE(m_Hume); 
  SR_SAFE_DELETE(m_Space); 
  SR_SAFE_DELETE(m_ground); 
} 
 
void Dyn_environment::Rendering_Fnc() 
{
  _Draw_Landing_Location();
  _Draw_Attraction_Location();
  _Draw_Foot_Force();
  _Draw_CoM_Height();
}
void Dyn_environment::_Draw_Landing_Location(){
  double radi(0.03);

  // Landing Location
  double height_landing_loc = Terrain_Interpreter::GetTerrainInterpreter()->surface_height( landing_loc_[0], landing_loc_[1]);
  glBegin(GL_LINE_LOOP); 
  double theta(0.0); 
  for (int i(0); i<3600; ++i){ 
    theta += DEG2RAD(i*0.1);
    glColor3f(0.5f, 0.1f, 0.5f); 
    glVertex3f(landing_loc_[0] + cos(theta)*radi, landing_loc_[1] + sin(theta)*radi, height_landing_loc);
  } 
  glEnd();
}
void Dyn_environment::_Draw_Attraction_Location(){
  // Attraction Location
  double radi(0.05);
  double theta(0.0);
  // double height_attraction_loc = Terrain_Interpreter::GetTerrainInterpreter()->surface_height( HUME_System::GetHumeSystem()->state_provider_->attraction_loc_[0],
  //                                                                                              HUME_System::GetHumeSystem()->state_provider_->attraction_loc_[1]);

  // glBegin(GL_LINE_LOOP); 
  // theta = 0.0; 
  // for (int i(0); i<3600; ++i){ 
  //     theta += DEG2RAD(i*0.1);
  //     glColor3f(0.5f, 0.1f, 0.f); 
  //     glVertex3f(HUME_System::GetHumeSystem()->state_provider_->attraction_loc_[0] + cos(theta)*radi,
  //                HUME_System::GetHumeSystem()->state_provider_->attraction_loc_[1] + sin(theta)*radi        , height_attraction_loc );
         
  // } 
  // glEnd(); 
}

void Dyn_environment::_Draw_CoM_Height(){
  double half_line_length(0.7);
  static int sparse(0);

  Vec3 CoM_pos;
  Vec3 CoM_des;
    
  // CoM_pos[0] = HUME_System::GetHumeSystem()->state_provider_->CoM_pos_[0]
  //     + ((Ctrl_Step*)(HUME_System::GetHumeSystem()->controller_))->walking_length_x_;
  // CoM_pos[1] = HUME_System::GetHumeSystem()->state_provider_->CoM_pos_[1]
  //     +  ((Ctrl_Step*)(HUME_System::GetHumeSystem()->controller_))->walking_length_y_;
  // CoM_pos[2] = HUME_System::GetHumeSystem()->state_provider_->CoM_pos_[2];

  // CoM_des = CoM_pos;

  // double base_pt_height(0.0);
  // if(  ((Ctrl_Step*)(HUME_System::GetHumeSystem()->controller_))->base_pt_ == LFOOT){
  //     base_pt_height = m_Hume->m_Link[HumeID::SIM_LFOOT].GetMassCenter()[2];
  // }
  // if(  ((Ctrl_Step*)(HUME_System::GetHumeSystem()->controller_))->base_pt_ == RFOOT){
  //     base_pt_height = m_Hume->m_Link[HumeID::SIM_RFOOT].GetMassCenter()[2];
  // }

    
  // CoM_des[2] = Terrain_Interpreter::GetTerrainInterpreter()->CoM_height(CoM_pos[0], CoM_pos[1]); // + base_pt_height;
    
  // if(sparse % 70 == 0){
  //     g_CoM_des_trajectory.push_back(CoM_des);
  // }

  // glColor3f(0.75f, 0.9f, 0.98f);
  // glBegin(GL_LINES);
  // std::vector<Vec3>::iterator iter(g_CoM_des_trajectory.end());
  // --iter;
  // int num_display(1500);
  // for (int i(0); i<num_display && iter != g_CoM_des_trajectory.begin(); ++i){
  //     glVertex3f((*iter)[0], 0.0 - half_line_length, (*iter)[2]);
  //     glVertex3f((*iter)[0], 0.0 + half_line_length, (*iter)[2]);
  //     --iter;
  // }
  // glEnd();
}

void Dyn_environment::_Draw_Foot_Force(){
  double small_ratio(0.02);

  // Right Foot Force 
  Vec3 right_foot = m_Hume->m_Link[HumeID::SIM_RFOOT].GetFrame().GetPosition(); 
  Vector right_force(3); 
  // for (int i(0); i<3; ++i){ 
  //     right_force[i] = right_foot[i] + small_ratio * HUME_System::GetHumeSystem()->state_provider_->curr_RFoot_force_[i]; 
  // } 
  // glColor3f(0.3f, 0.1f, 0.1f); 
  // glBegin(GL_LINES); 
  // glVertex3f(right_foot[0],  right_foot[1],  right_foot[2]); 
  // glVertex3f(right_force[0], right_force[1], right_force[2]); 
  // glEnd(); 
 
  // // Left Foot Force 
  // Vec3 left_foot = m_Hume->m_Link[HumeID::SIM_LFOOT].GetFrame().GetPosition(); 
  // Vector left_force(3); 
  // for (int i(0); i<3; ++i){ 
  //     left_force[i] = left_foot[i] + small_ratio * HUME_System::GetHumeSystem()->state_provider_->curr_LFoot_force_[i]; 
  // } 
  // glColor3f(0.3f, 0.1f, 0.1f); 
  // glBegin(GL_LINES); 
  // glVertex3f(left_foot[0],  left_foot[1],  left_foot[2]); 
  // glVertex3f(left_force[0], left_force[1], left_force[2]); 
  // glEnd();
}

void Dyn_environment::_Draw_Foot_Body_Trajectory(){
  // Body & Foot Trajectory
  glColor3f(0.5f, 0.1f, 0.1f); 
  glBegin(GL_LINE_STRIP); 
  for (unsigned int i(0); i<g_trajectory.size(); ++i) 
    { 
      glVertex3f(g_trajectory[i][0], g_trajectory[i][1], g_trajectory[i][2]); 
    } 
  glEnd(); 
 
  glColor3f(0.5f, 1.1f, 0.1f); 
  glBegin(GL_LINE_STRIP); 
  for (unsigned int i(0); i<g_leftfoot.size(); ++i) 
    { 
      glVertex3f(g_leftfoot[i][0], 
                 g_leftfoot[i][1], 
                 g_leftfoot[i][2]); 
    } 
  glEnd(); 
 
  glColor3f(0.5f, 0.1f, 1.1f); 
  glBegin(GL_LINE_STRIP); 
  for (unsigned int i(0); i<g_rightfoot.size(); ++i) 
    { 
      glVertex3f(g_rightfoot[i][0], 
                 g_rightfoot[i][1], 
                 g_rightfoot[i][2]); 
    } 
  glEnd();
}
void Dyn_environment::_Hold_Rx(Hume* hume){
  static int _count(0);
  ++_count;
  double kp(250.0);
  double kd(30.0);

  double omega(0.0);
  double rx_des = 0.2*sin(omega*((double)(_count*SERVO_RATE)));
  double rx_vel_des = 0.2*omega*cos(omega * ((double)(_count*SERVO_RATE)));

    
  double rx_angle   ( hume->Full_Joint_State_[3+ HumeID::SIM_Rx]->m_rValue[0]); 
  double rx_ang_vel ( hume->Full_Joint_State_[3+ HumeID::SIM_Rx]->m_rValue[1]);
  hume->Full_Joint_State_[3 + HumeID::SIM_Rx]->m_rCommand =
    +kp*(rx_des - rx_angle) + kd * (rx_vel_des- rx_ang_vel); 
}

void Dyn_environment::_Hold_Ry(Hume* hume){
  static int _count(0);
  ++_count;
  double kp(230.);
  double kd(30.);

  // double omega = 2.5;
  double omega(0.0);
  double ry_des = 0.2*sin(omega*((double)(_count*SERVO_RATE)));
  double ry_vel_des = 0.2*omega*cos(omega * ((double)(_count*SERVO_RATE)));
    
  double ry_angle   ( hume->Full_Joint_State_[3+ HumeID::SIM_Ry]->m_rValue[0]); 
  double ry_ang_vel ( hume->Full_Joint_State_[3+ HumeID::SIM_Ry]->m_rValue[1]);
  hume->Full_Joint_State_[3 + HumeID::SIM_Ry]->m_rCommand =
    kp*(ry_des - ry_angle) + kd * (ry_vel_des - ry_ang_vel); 
}

void Dyn_environment::_Hold_Rz(Hume* hume){
  double kp(200.0);
  double kd(10.0);
    
  double rz_angle   ( hume->Full_Joint_State_[3+ HumeID::SIM_Rz]->m_rValue[0]); 
  double rz_ang_vel ( hume->Full_Joint_State_[3+ HumeID::SIM_Rz]->m_rValue[1]);
  hume->Full_Joint_State_[3 + HumeID::SIM_Rz]->m_rCommand =
    +kp*(DEG2RAD(0.0) - rz_angle) - kd * rz_ang_vel; 
}

void Dyn_environment::_Hold_X(Hume* hume){
  double kp(2500.0);
  double kd(50.0);
    
  double x_dis (hume->Full_Joint_State_[HumeID::SIM_X]->m_rValue[0]); 
  double x_vel (hume->Full_Joint_State_[HumeID::SIM_X]->m_rValue[1]); 
  hume->Full_Joint_State_[HumeID::SIM_X]->m_rCommand =
    -kp * x_dis - kd * x_vel; 
}

void Dyn_environment::_Hold_Y(Hume* hume){
  double kp(2500.0);
  double kd(50.0);

    
  double y_dis (hume->Full_Joint_State_[HumeID::SIM_Y]->m_rValue[0]); 
  double y_vel (hume->Full_Joint_State_[HumeID::SIM_Y]->m_rValue[1]); 
  hume->Full_Joint_State_[HumeID::SIM_Y]->m_rCommand =
    -kp*y_dis - kd * y_vel; 
}

void Dyn_environment::_Hold_Z(Hume* hume){
  double z_dis (hume->Full_Joint_State_[HumeID::SIM_Z]->m_rValue[0]); 
  double z_vel (hume->Full_Joint_State_[HumeID::SIM_Z]->m_rValue[1]); 
  hume->Full_Joint_State_[HumeID::SIM_Z]->m_rCommand =
    -500.0*z_dis - 200.0 * z_vel; 
}

void Dyn_environment::_Torque_Controller(Dyn_environment* dyn_env,
                                         const std::vector<int32_t> & kp_torque,
                                         const std::vector<int32_t> & ki_torque,
                                         const std::vector<double> & command){
  static std::vector<double> error_sum(6, 0.0);
  double cramp(30.0);
  for (int i(0); i<SIM_NUM_RJOINT; ++i){
    error_sum[i] += (-command[i] - dyn_env->torque_[i]) * SERVO_RATE;

    if(error_sum[i]> cramp){
      error_sum[i] = cramp;
    }
    else if (error_sum[i] < -cramp){
      error_sum[i] = -cramp;
    }
  }
    
  for(int i(0); i<SIM_NUM_RJOINT; ++i){ 
    dyn_env->m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rCommand =
      -0.4 * command[i] 
      + 0.0012 * fabs(kp_torque[i])*(-command[i] - dyn_env->torque_[i])
      + 0.0007 *fabs(ki_torque[i])*error_sum[i]
      - 0.012 *dyn_env->curr_jvel_[i]; 
  }
  int i(0);
  dyn_env->m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rCommand =
    -0.35 * command[i] 
    + 0.0008 * fabs(kp_torque[i])*(-command[i] - dyn_env->torque_[i])
    + 0.0003 *fabs(ki_torque[i])*error_sum[i]
    - 0.012 *dyn_env->curr_jvel_[i];

  i = 3;
  dyn_env->m_Hume->Full_Joint_State_[i + SIM_NUM_PASSIVE]->m_rCommand =
    -0.35 * command[i] 
    + 0.0008 * fabs(kp_torque[i])*(-command[i] - dyn_env->torque_[i])
    + 0.0003 *fabs(ki_torque[i])*error_sum[i]
    - 0.012 *dyn_env->curr_jvel_[i]; 
}
