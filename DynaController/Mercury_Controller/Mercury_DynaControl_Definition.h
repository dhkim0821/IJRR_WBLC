#ifndef MERCURY_DYNACORE_CONTROL_DEFINITION
#define MERCURY_DYNACORE_CONTROL_DEFINITION

#include <Configuration.h>
#include <Mercury/Mercury_Definition.h>

#define MercuryConfigPath THIS_COM"DynaController/Mercury_Controller/MercuryTestConfig/"

namespace base_condition{
    constexpr int fixed = 0;
    constexpr int floating = 1;
    constexpr int lying = 2;
    constexpr int holding = 3;
};

class Mercury_SensorData{
    public:
        double imu_inc[3];
        double imu_ang_vel[3];
        double imu_acc[3];
        double joint_jpos[mercury::num_act_joint];
        double joint_jvel[mercury::num_act_joint];
        double motor_jpos[mercury::num_act_joint];
        double motor_jvel[mercury::num_act_joint];
        double bus_current[mercury::num_act_joint];
        double bus_voltage[mercury::num_act_joint];
        double jtorque[mercury::num_act_joint];
        double motor_current[mercury::num_act_joint];
        double reflected_rotor_inertia[mercury::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class Mercury_Command{
    public:
        double jtorque_cmd[mercury::num_act_joint];
        double jpos_cmd[mercury::num_act_joint];
        double jvel_cmd[mercury::num_act_joint];
};

#endif
