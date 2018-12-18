#ifndef ATLAS_INTERFACE_H
#define ATLAS_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "Atlas_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class Atlas_StateEstimator;
class Atlas_StateProvider;

class Atlas_interface: public interface{
    public:
        Atlas_interface();
        virtual ~Atlas_interface();
        virtual void GetCommand(void * data, void * command);

    private:
        int waiting_count_;

        void _ParameterSetting();
        bool _Initialization(Atlas_SensorData* );

        Atlas_Command* test_cmd_;
        dynacore::Vector initial_upper_body_config_;

        dynacore::Vector jjvel_;
        dynacore::Vector jjpos_;
        dynacore::Vector jtorque_;

        dynacore::Vector torque_command_;
        dynacore::Vector jpos_command_;
        dynacore::Vector jvel_command_;

        Atlas_StateEstimator* state_estimator_;
        Atlas_StateProvider* sp_;
};

#endif
