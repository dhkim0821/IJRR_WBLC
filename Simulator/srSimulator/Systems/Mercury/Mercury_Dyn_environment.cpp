#include "Mercury_Dyn_environment.hpp"
#include "common/utils.h"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <DynaController/Mercury_Controller/Mercury_StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include "LED_Position_Announcer.hpp"
// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* mercury::servo_rate (sec) = time delay 


 

Mercury_Dyn_environment::Mercury_Dyn_environment():
  num_substep_rendering_(15),
  count_(0)
{
  _ParamterSetup();

  m_Mercury = new Mercury(Vec3(0.0, 0.0, 0.0), srSystem::FIXED, srJoint::TORQUE);
  m_Space = new srSpace();
  m_ground = new Ground();

  interface_ = new Mercury_interface();
  data_ = new Mercury_SensorData();
  cmd_ = new Mercury_Command();

  sp_ = Mercury_StateProvider::getStateProvider();

  m_Space->AddSystem(m_ground->BuildGround());
  m_Space->AddSystem((srSystem*)m_Mercury);
  m_Space->DYN_MODE_PRESTEP();

  m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
  m_Space->SetTimestep(mercury::servo_rate);
  m_Space->SetGravity(0.0,0.0,-9.81);
  m_Space->SetNumberofSubstepForRendering(num_substep_rendering_);

  led_pos_announcer_ = new LED_Position_Announcer(this);
  led_pos_announcer_->start();
  // Initialize differentiation values
  prev_imu_pos.clear();  cur_imu_pos.clear();
  prev_imu_vel.clear();  cur_imu_vel.clear();
  cur_imu_acc.clear();
  for(int i = 0; i < 3; i++){
    prev_imu_pos.push_back(0.0);  cur_imu_pos.push_back(0.0);  
    prev_imu_vel.push_back(0.0); cur_imu_vel.push_back(0.0);
    cur_imu_acc.push_back(0.0);  
  }
}

void Mercury_Dyn_environment::ContolFunction( void* _data ) {
  Mercury_Dyn_environment* pDyn_env = (Mercury_Dyn_environment*)_data;
  Mercury* robot = pDyn_env->m_Mercury;
  Mercury_SensorData* p_data = pDyn_env->data_;
  ++pDyn_env->count_ ;
  double alternate_time = mercury::servo_rate * pDyn_env->count_;
  std::vector<double> jpos(6);
  std::vector<double> mjvel(6);
  std::vector<double> jjvel(6);
  std::vector<double> jtorque(6);
  std::vector<double> imu_acc(3);
  std::vector<double> imu_ang_vel(3);
  std::vector<double> imu_inc(3);
  bool rfoot_contact(false);
  bool lfoot_contact(false);
  std::vector<double> torque_command(robot->num_act_joint_, 0.);

  pDyn_env->getIMU_Data(imu_acc, imu_ang_vel);

  for(int i(0); i<3; ++i){
      p_data->imu_ang_vel[i] = imu_ang_vel[i];
      p_data->imu_acc[i] = -imu_acc[i];
      p_data->imu_inc[i] = -imu_acc[i];
  }

  // Simulated foot contact data
  pDyn_env->getFootContact_Data(lfoot_contact, rfoot_contact);
  p_data->lfoot_contact = lfoot_contact;
  p_data->rfoot_contact = rfoot_contact;  


  int lj_start_idx(4);
  // Right
  for(int i(0); i< 3; ++i){
    p_data->joint_jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
    p_data->motor_jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
    p_data->motor_jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    p_data->joint_jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    p_data->jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
  }
  // Left
  for(int i(0); i< 3; ++i){
    p_data->joint_jpos[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[0];
    p_data->motor_jpos[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[0];    
    p_data->motor_jvel[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[1];
    p_data->joint_jvel[i+3] = robot->r_joint_[i + lj_start_idx]->m_State.m_rValue[1];
    p_data->jtorque[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[3];
  }

  pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_);

  for(int i(0); i<3; ++i){
    robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
    robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
  }
  double Kp(300.0);
  double Kd(50.0);
  double K_friction(0.0);
  // double Kp(30.);
  // double Kd(0.5);
  // Right
  for(int i(0); i<3; ++i){
    robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
        Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->joint_jpos[i]) + 
        Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->motor_jvel[i]) + 
        K_friction * (0. - p_data->motor_jvel[i]);
  }
  // Left
  for(int i(0); i<3; ++i){
    robot->r_joint_[i+lj_start_idx]->m_State.m_rCommand = 
        pDyn_env->cmd_->jtorque_cmd[i+3] + 
         Kp * (pDyn_env->cmd_->jpos_cmd[i+3] - p_data->joint_jpos[i+3]) + 
        Kd * (pDyn_env->cmd_->jvel_cmd[i+3] - p_data->motor_jvel[i+3]) + 
        K_friction * (0. - p_data->motor_jvel[i + 3]);
  }

  // Ankle Passive
  pDyn_env->PassiveAnkleSpring();

  // Push Body
  pDyn_env->PushRobotBody();

  // Hold Body
  if(pDyn_env->count_*mercury::servo_rate < pDyn_env->release_time_){
    pDyn_env->FixXY();
    pDyn_env->FixRxRy();
  }
}

void Mercury_Dyn_environment::PushRobotBody(){
  static int push_idx(0);
  if(push_time_.size() > push_idx){

    if(count_ * mercury::servo_rate > push_time_[push_idx]){
      int body_lk_idx(m_Mercury->link_idx_map_.find("body")->second);
      // Force Setting
      double force_size(push_force_[push_idx]);
      double force_dir(DEG2RAD(push_direction_[push_idx]));

      double force_x = -force_size * cos(force_dir);
      double force_y = -force_size * sin(force_dir);

      dse3 ext_force(0., 0., 0.,force_x, force_y, 0.);
      m_Mercury->link_[body_lk_idx]->AddUserExternalForce(ext_force);

      if(count_*mercury::servo_rate > push_time_[push_idx] + 0.1){
        ++push_idx;
      }
    }
  }
}
void Mercury_Dyn_environment::PassiveAnkleSpring(){
  double kp(0.02);
  double kd(0.005);
  double des_pos(-0.5);
  m_Mercury->r_joint_[3]->m_State.m_rCommand = kp * (des_pos - m_Mercury->r_joint_[3]->m_State.m_rValue[0]) - kd* m_Mercury->r_joint_[3]->m_State.m_rValue[1];
  m_Mercury->r_joint_[7]->m_State.m_rCommand = kp * (des_pos - m_Mercury->r_joint_[7]->m_State.m_rValue[0]) - kd* m_Mercury->r_joint_[7]->m_State.m_rValue[1];
}

void Mercury_Dyn_environment::FixXY(){
  double pos,vel;

  double kp(1500.0);
  double kd(150.0);

  int idx(0);
  pos = m_Mercury->vp_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vp_joint_[idx]->m_State.m_rValue[1];
  m_Mercury->vp_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

  idx = 1;
  pos = m_Mercury->vp_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vp_joint_[idx]->m_State.m_rValue[1];

  m_Mercury->vp_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;
}

void Mercury_Dyn_environment::FixRxRy(){
  double pos,vel;

  double kp(500.0);
  double kd(15);

  int idx(1);
  pos = m_Mercury->vr_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vr_joint_[idx]->m_State.m_rValue[1];
  m_Mercury->vr_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

  idx = 2;
  pos = m_Mercury->vr_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vr_joint_[idx]->m_State.m_rValue[1];

  m_Mercury->vr_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;
}


void Mercury_Dyn_environment::Rendering_Fnc(){
    _DrawDesiredLocation();
}
void Mercury_Dyn_environment::_DrawDesiredLocation(){
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

void Mercury_Dyn_environment::_ParamterSetup(){
  ParamHandler handler(MercuryConfigPath"SIM_sr_sim_setting.yaml");
  handler.getInteger("num_substep_rendering", num_substep_rendering_);
  handler.getValue("releasing_time", release_time_);
  handler.getVector("imu_angular_velocity_bias", imu_ang_vel_bias_);
  handler.getVector("imu_angular_velocity_noise_variance", imu_ang_vel_var_);

  handler.getVector("push_timing", push_time_);
  handler.getVector("push_force", push_force_);
  handler.getVector("push_direction", push_direction_);
}

void Mercury_Dyn_environment::getFootContact_Data(bool & left_foot_contact,
                                                  bool & right_foot_contact){

  // Check for foot contact
  SE3 lfoot_frame = m_Mercury->link_[m_Mercury->link_idx_map_.find("lfoot")->second]->GetFrame();
  SE3 rfoot_frame = m_Mercury->link_[m_Mercury->link_idx_map_.find("rfoot")->second]->GetFrame();

  // std::cout << "lfoot(" << 2 << ",3) = " << lfoot_frame(2,3) << std::endl;
  // std::cout << "rfoot(" << 2 << ",3) = " << rfoot_frame(2,3) << std::endl;     
  // Contact occurs when lfoot_z is less than 0.022
  double ground_contact_height = 0.0227;

  if (lfoot_frame(2,3) <= ground_contact_height){
    left_foot_contact = true;
  }else{
    left_foot_contact = false;
  }
  if (rfoot_frame(2,3) <= ground_contact_height){
    right_foot_contact = true;
  } else{
    right_foot_contact = false;
  }

}


void Mercury_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetVel();
  se3 imu_se3_acc = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetAcc();
  SE3 imu_frame = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetFrame();
  SO3 imu_ori = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetOrientation();

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
    imu_acc[i] = imu_se3_acc[i+3] - local_grav[i];
  }

  Eigen::Matrix<double, 3, 1> ang_vel;
  ang_vel<<imu_se3_vel[0], imu_se3_vel[1], imu_se3_vel[2];
  dynacore::Vect3 global_ang_vel = Rot * ang_vel;
  Eigen::Quaterniond quat(Rot);

  static int count = 0;

  static double pos_x_bias = imu_frame(0,3);
  static double pos_y_bias = imu_frame(1,3);
  static double pos_z_bias = imu_frame(2,3);

  std::vector<double> pos_bias; pos_bias.push_back(pos_x_bias); pos_bias.push_back(pos_y_bias); pos_bias.push_back(pos_z_bias);

  for(size_t i = 0; i < 3; i++){
    sp_->sim_imu_pos[i] = imu_frame(i,3) - pos_bias[i];
    sp_->sim_imu_vel[i] = imu_se3_vel[i+3];
  }


  // Compute differentiated values -------------------------------
  for(size_t i = 0; i < 3; i++){
    // Compute new x, dx, ddx values
    cur_imu_pos[i] = imu_frame(i,3) - pos_bias[i];
    cur_imu_vel[i] = (cur_imu_pos[i] - prev_imu_pos[i])/mercury::servo_rate;
    cur_imu_acc[i] = ((cur_imu_vel[i] - prev_imu_vel[i])/mercury::servo_rate) - local_grav[i] ;
    // Update previous values
    prev_imu_pos[i] = cur_imu_pos[i];
    prev_imu_vel[i] = cur_imu_vel[i];    
  }
  // Use numerically computed acceleration values:
  for(size_t i = 0; i < 3; i++){
    imu_acc[i] = cur_imu_acc[i];
  }





  bool b_printout(false);
  if (count % 100 == 0){
    if(b_printout){
      printf("imu info: \n");
      std::cout<<imu_se3_vel<<std::endl;
      std::cout<<imu_se3_acc<<std::endl;
      std::cout<<imu_frame<<std::endl;

      dynacore::pretty_print(imu_ang_vel, "imu ang vel");
      dynacore::pretty_print(imu_acc, "imu_acc");

      dynacore::pretty_print(cur_imu_pos, "sim imu pos");
      dynacore::pretty_print(cur_imu_vel, "diff imu vel");
      dynacore::pretty_print(cur_imu_acc, "diff imu acc");


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
