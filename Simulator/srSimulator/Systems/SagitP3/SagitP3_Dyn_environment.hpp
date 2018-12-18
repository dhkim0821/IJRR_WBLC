#ifndef  DYN_ENVIRONMENT_SagitP3
#define  DYN_ENVIRONMENT_SagitP3

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>
#include "SagitP3.hpp"

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
class SagitP3_Command;
class SagitP3_SensorData;
class SagitP3_StateData;

class SagitP3_Dyn_environment
{
    public:
        SagitP3_Dyn_environment();
        ~SagitP3_Dyn_environment();

        static void ControlFunction(void* _data);
        void Rendering_Fnc();

        void SetCurrentState_All();
        void saveLandingLocation();
    public:
        SagitP3_SensorData* data_;
        SagitP3_StateData* state_data_;
        SagitP3_Command* cmd_;

        interface* interface_;
        interface* state_interface_;
        SagitP3* robot_;

        srSpace*	m_Space;
        Ground*	m_ground;

        double ori_mtx_[9];
        std::vector<double> ang_vel_  ;
    protected:
        void getIMU_Data(std::vector<double> & imu_acc,
                std::vector<double> & imu_ang_vel);
        void _Get_Orientation(dynacore::Quaternion & rot, dynacore::Vect3 &ang_vel);
        void _Copy_Array(double * , double *, int);
        void _CheckFootContact(bool & r_contact, bool & l_contact);
        void _hold_Ori(int count);
        void _hold_XY(int count);
        void _ZeroInput_VirtualJoint();

        void _ParamterSetup();
        int num_substep_rendering_;
        double release_time_;
        std::vector<double> imu_ang_vel_bias_;
        std::vector<double> imu_ang_vel_var_;
};

#endif
