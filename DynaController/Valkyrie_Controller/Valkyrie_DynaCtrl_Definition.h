#ifndef VALKYRIE_DYNACORE_CONTROL_DEFINITION
#define VALKYRIE_DYNACORE_CONTROL_DEFINITION

#include <Valkyrie/Valkyrie_Definition.h>

#define ValkyrieConfigPath THIS_COM"DynaController/Valkyrie_Controller/ValkyrieTestConfig/"

class Valkyrie_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];
        double jpos[valkyrie::num_act_joint];
        double jvel[valkyrie::num_act_joint];
        double jtorque[valkyrie::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class Valkyrie_Command{
    public:
        double jtorque_cmd[valkyrie::num_act_joint];
        double jpos_cmd[valkyrie::num_act_joint];
        double jvel_cmd[valkyrie::num_act_joint];
};


#endif
