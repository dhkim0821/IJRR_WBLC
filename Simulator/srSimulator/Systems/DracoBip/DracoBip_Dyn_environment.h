#ifndef  DYN_ENVIRONMENT_DracoBip
#define  DYN_ENVIRONMENT_DracoBip

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>
#include "DracoBip.h"
#include <DynaController/DracoBip_Controller/DracoBip_StateProvider.hpp>


//TEST

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
class DracoBip_Command;
class DracoBip_SensorData;

class DracoBip_Dyn_environment
{
    public:
        DracoBip_Dyn_environment();
        ~DracoBip_Dyn_environment();

        static void ControlFunction(void* _data);
        void Rendering_Fnc();

        void SetCurrentState_All();
        void saveLandingLocation();
        void PushRobotBody();
    public:
        DracoBip_SensorData* data_;
        DracoBip_Command* cmd_;

        interface* interface_;
        DracoBip* robot_;

        DracoBip_StateProvider* sp_;

        srSpace*	m_Space;
        Ground*	m_ground;

        double ori_mtx_[9];
        std::vector<double> ang_vel_  ;
        int count_;
        double simulation_freq_;

    protected:
        void getIMU_Data(std::vector<double> & imu_acc,
                std::vector<double> & imu_ang_vel);
         void _Get_Orientation(dynacore::Quaternion & rot);
        void _Copy_Array(double * , double *, int);
        void _CheckFootContact(bool & r_contact, bool & l_contact);
        void _hold_XY();
        void _hold_Orientation();
        void _ZeroInput_VirtualJoint();
        void _ParamterSetup();

        void _DrawDesiredLocation();

        std::vector<double> push_time_;
        std::vector<double> push_force_;
        std::vector<double> push_direction_;

        int num_substep_rendering_;
        double release_time_;
        std::vector<double> imu_ang_vel_bias_;
        std::vector<double> imu_ang_vel_var_;
};

#endif
