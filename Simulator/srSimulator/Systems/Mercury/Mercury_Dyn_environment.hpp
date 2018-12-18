#ifndef		MERCURY_DYN_ENVIRONMENT
#define		MERCURY_DYN_ENVIRONMENT

#include "Mercury.hpp"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "Ground.h"
#include <vector>
#include "Configuration.h"
#include <DynaController/Mercury_Controller/Mercury_interface.hpp>
#include <DynaController/Mercury_Controller/Mercury_StateProvider.hpp>

//////////////////////////////////////////////// 
#ifdef __APPLE__ 
#include <GLUT/glut.h> 
#endif 
 
#ifdef __linux__ 
#include <GL/glut.h> 
#endif 
//////////////////////////////////////////////// 

class interface;
class LED_Position_Announcer;

class Mercury_Dyn_environment
{
public:
  Mercury_Dyn_environment();
  ~Mercury_Dyn_environment();

  static void ContolFunction(void* _data);

  int count_;

  void Rendering_Fnc();
  void FixXY();
  void FixRxRy();
  void PassiveAnkleSpring();
  void PushRobotBody();
  
  interface* interface_;

  Mercury_StateProvider* sp_;

  Mercury_SensorData* data_;
  Mercury_Command* cmd_;
  Mercury*	m_Mercury;
  srSpace*	m_Space;
  Ground*	m_ground;
    LED_Position_Announcer* led_pos_announcer_;

  void getIMU_Data(std::vector<double> & imu_acc,
                   std::vector<double> & imu_ang_vel);

  void getFootContact_Data(bool & left_foot_contact,
                           bool & right_foot_contact);  

private:

  std::vector<double> push_time_;
  std::vector<double> push_force_;
  std::vector<double> push_direction_;


  // Numerical differentiation of position and velocity
  std::vector<double> prev_imu_pos;
  std::vector<double> cur_imu_pos;  
  std::vector<double> prev_imu_vel;
  std::vector<double> cur_imu_vel;  
  std::vector<double> cur_imu_acc;  


  
  void _DrawDesiredLocation();
  void _ParamterSetup();
  int num_substep_rendering_;
  double release_time_;
  std::vector<double> imu_ang_vel_bias_;
  std::vector<double> imu_ang_vel_var_;
};

#endif
