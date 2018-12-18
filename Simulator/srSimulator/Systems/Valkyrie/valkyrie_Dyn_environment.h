#ifndef  DYN_ENVIRONMENT_Valkyrie
#define  DYN_ENVIRONMENT_Valkyrie

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>
#include "new_valkyrie.h"

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
class Valkyrie_Command;
class Valkyrie_SensorData;
class srSpace;
class Ground;
class Valkyrie_StateProvider;

class Valkyrie_Dyn_environment
{
public:
  Valkyrie_Dyn_environment();
  ~Valkyrie_Dyn_environment();

  static void ControlFunction(void* _data);
  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
  Valkyrie_SensorData* data_;
  Valkyrie_Command* cmd_;

  interface* interface_;
  New_Valkyrie* robot_;

  srSpace*	m_Space;
  Ground*	m_ground;

  double ori_mtx_[9];
  std::vector<double> ang_vel_  ;
  void getIMU_Data(std::vector<double> & imu_acc,
          std::vector<double> & imu_ang_vel);
 
  Valkyrie_StateProvider* sp_;
protected:
 void _SaveStanceFoot();
  void _ExternalDisturbance(int count);

  void _Save_Orientation_Matrix();
  void _Get_Orientation(dynacore::Quaternion & rot);

  void _Copy_Array(double * , double *, int);

  void _ListReactionForce();
  void _ListCommandedReactionForce(const dynacore::Vector & Fr);
  void _DrawDesiredLocation();
  void _Draw_Contact_Point();
  void _Draw_Contact_Force();
  void _Draw_Commanded_Force();
  void _DrawHollowCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);
  void _CheckFootContact(bool & r_contact, bool & l_contact);
      //void _Draw_FootPlacement();
};

#endif
