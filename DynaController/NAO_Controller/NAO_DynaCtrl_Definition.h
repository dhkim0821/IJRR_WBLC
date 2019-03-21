#ifndef NAO_DYNACORE_CONTROL_DEFINITION
#define NAO_DYNACORE_CONTROL_DEFINITION

#include <NAO/NAO_Definition.h>

#define NAOConfigPath THIS_COM"DynaController/NAO_Controller/NAOTestConfig/"

class NAO_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];
        double jpos[nao::num_act_joint];
        double jvel[nao::num_act_joint];
        double jtorque[nao::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class NAO_Command{
    public:
        double jtorque_cmd[nao::num_act_joint];
        double jpos_cmd[nao::num_act_joint];
        double jvel_cmd[nao::num_act_joint];
};


#endif
