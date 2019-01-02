#ifndef  DYN_ENVIRONMENT_Atlas
#define  DYN_ENVIRONMENT_Atlas

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>
#include <DynaController/Atlas_Controller/Atlas_StateProvider.hpp>
//TEST JUNHYEOK
#include "Atlas.h"

//TEST
////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

//#ifdef linux
#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////


class interface;
class srSpace;
class Ground;
class Atlas_Command;
class Atlas_SensorData;

class Atlas_Dyn_environment
{
public:
  Atlas_Dyn_environment();
  ~Atlas_Dyn_environment();

  static void ControlFunction(void* _data);
  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
   Atlas_SensorData* data_;
   Atlas_Command* cmd_;

  interface* interface_;
  Atlas* robot_;

  Atlas_StateProvider* sp_;

  srSpace*	m_Space;
  Ground*	m_ground;

  double ori_mtx_[9];
  std::vector<double> ang_vel_  ;
  void getIMU_Data(std::vector<double> & imu_acc,
          std::vector<double> & imu_ang_vel);
 protected:
  void _DrawDesiredLocation();
  void _Get_Orientation(dynacore::Quaternion & rot);
  void _Copy_Array(double * , double *, int);
  void _CheckFootContact(bool & r_contact, bool & l_contact);
};

#endif
