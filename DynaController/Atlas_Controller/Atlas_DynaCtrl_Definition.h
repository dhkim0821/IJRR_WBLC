#ifndef ATLAS_DYNACORE_CONTROL_DEFINITION
#define ATLAS_DYNACORE_CONTROL_DEFINITION

#include <Atlas/Atlas_Definition.h>

#define AtlasConfigPath THIS_COM"DynaController/Atlas_Controller/AtlasTestConfig/"

class Atlas_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];
        double jpos[atlas::num_act_joint];
        double jvel[atlas::num_act_joint];
        double jtorque[atlas::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class Atlas_Command{
    public:
        double jtorque_cmd[atlas::num_act_joint];
        double jpos_cmd[atlas::num_act_joint];
        double jvel_cmd[atlas::num_act_joint];
};


#endif
