#ifndef  DYN_ENVIRONMENT_DRACO
#define  DYN_ENVIRONMENT_DRACO

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>

////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////


class Interface;
class srSpace;
class Ground;
class srDraco;

struct state
{
    std::vector<double> conf;
    std::vector<double> jvel;
    std::vector<double> torque   ;
    // std::vector<double> euler_ang;
    double ori_mtx[9];
    std::vector<double> ang_vel;
};

class Draco_Dyn_environment
{
    public:
        Draco_Dyn_environment();
        ~Draco_Dyn_environment();

        static void ContolFunction(void* _data);

        void Rendering_Fnc();

        void SetCurrentState_All();
        void saveLandingLocation();
    public:
        Interface* interface_;
        srDraco* robot_;

        srSpace*	m_Space;
        Ground*	m_ground;

        std::vector<double> curr_conf_;
        std::vector<double> curr_jvel_;
        std::vector<double> torque_   ;

        double ori_mtx_[9];
        std::vector<double> ang_vel_  ;

        std::vector<state> history_state_;
        std::vector<sejong::Vect3> contact_pt_list_;
        std::vector<sejong::Vect3> contact_force_list_;

        std::vector<sejong::Vect3> indicated_contact_pt_list_;
        std::vector<sejong::Vect3> commanded_contact_force_list_;

    protected:
        void _PushBody(const int & curr_count,
                       double Fx, double Fy, double Fz,
                       int start_count, int end_count);

        void _Save_Orientation_Matrix();
        void _Get_Orientation(sejong::Quaternion & rot);

        void _Copy_Array(double * , double *, int);

        void _ListCommandedReactionForce(const sejong::Vector & Fr);
        void _Draw_Commanded_Force();
        void _Draw_Contact_Point();
};

#endif
