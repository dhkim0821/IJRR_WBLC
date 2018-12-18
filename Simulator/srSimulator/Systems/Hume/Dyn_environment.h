#ifndef		_DYN_ENVIRONMENT_
#define		_DYN_ENVIRONMENT_

#include "Hume.h"
#include "Terrain.h"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "Ground.h"
#include "LieGroup/LieGroup.h"
#include <vector>
#include <ControlSystem/Hume_Controller/HumeSystem.h>
#include "Configuration.h"
#include "simulation_setting.h"
#include "LED_Position_Announcer.h"

struct state
{
    std::vector<double> conf;
    std::vector<double> jvel;
    std::vector<double> torque   ;
    // std::vector<double> euler_ang;
    double ori_mtx[9];
    std::vector<double> ang_vel;
    bool left_foot_contact;
    bool right_foot_contact;
    std::vector<double> accelerometer;
};

class Dyn_environment
{
public:
    Dyn_environment();
    ~Dyn_environment();

    static void ContolFunction(void* _data);

    static void _Hold_Rx(Hume* );
    static void _Hold_Ry(Hume* );
    static void _Hold_Rz(Hume* );

    static void _Hold_X(Hume* );
    static void _Hold_Y(Hume* );
    static void _Hold_Z(Hume* );
    static void _UpdateStateProvider(Hume* );

    static void _Torque_Controller(Dyn_environment * dyn_env,
                                   const std::vector<int32_t> & kp_torque,
                                   const std::vector<int32_t> & ki_torque,
                                   const std::vector<double> & command);
    
    void Rendering_Fnc();
    void _Draw_Foot_Body_Trajectory();
    void _Draw_Foot_Force();
    void _Draw_Attraction_Location();
    void _Draw_Landing_Location();
    void _Draw_CoM_Height();
    void SetCurrentState_All();
    void saveLandingLocation();
public:
    Hume*	m_Hume;
    Terrain* terrain_;
    srSpace*	m_Space;
    Ground*	m_ground;
    Controller_Hume*  controller_;
    Vector landing_loc_;

    std::vector<double> curr_conf_;
    std::vector<double> curr_jvel_;
    std::vector<double> torque_   ;
    // std::vector<double> euler_ang_;
    double ori_mtx_[9];
    Eigen::Matrix3d sj_ori_mtx_;
    
    std::vector<double> ang_vel_  ;
    std::vector<double> acc_;

    bool left_foot_contact_ ;
    bool right_foot_contact_;

    std::vector<state> history_state_;

    LED_Position_Announcer* LED_pos_announcer_;

protected:
    void _Save_Orientation_Matrix();
    void _Copy_Array(double * , double *, int);
};

#endif
