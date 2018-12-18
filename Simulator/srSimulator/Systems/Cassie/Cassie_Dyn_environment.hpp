#ifndef  DYN_ENVIRONMENT_Cassie
#define  DYN_ENVIRONMENT_Cassie

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>
#include "Cassie.hpp"

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
class Cassie_Command;
class Cassie_SensorData;

class Cassie_Dyn_environment
{
    public:
        Cassie_Dyn_environment();
        ~Cassie_Dyn_environment();

        static void ControlFunction(void* _data);
        void Rendering_Fnc();

        void SetCurrentState_All();
        void saveLandingLocation();
    public:
        Cassie_SensorData* data_;
        Cassie_Command* cmd_;

        interface* interface_;
        Cassie* robot_;

        srSpace*	m_Space;
        Ground*	m_ground;

        double ori_mtx_[9];
        std::vector<double> ang_vel_  ;
    protected:
        void _Get_Orientation(dynacore::Quaternion & rot);
        void _Copy_Array(double * , double *, int);
        void _CheckFootContact(bool & r_contact, bool & l_contact);
        void _hold_XY(int count);
        void _ZeroInput_VirtualJoint();

        void _ParamterSetup();
        int num_substep_rendering_;
        double release_time_;
        std::vector<double> imu_ang_vel_bias_;
        std::vector<double> imu_ang_vel_var_;
};

#endif
