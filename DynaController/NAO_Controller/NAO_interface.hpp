#ifndef NAO_INTERFACE_H
#define NAO_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "NAO_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class NAO_StateEstimator;
class NAO_StateProvider;

class NAO_interface: public interface{
    public:
        NAO_interface();
        virtual ~NAO_interface();
        virtual void GetCommand(void * data, void * command);

    private:
        int waiting_count_;

        void _ParameterSetting();
        bool _Initialization(NAO_SensorData* );

        NAO_Command* test_cmd_;
        dynacore::Vector initial_upper_body_config_;

        dynacore::Vector jjvel_;
        dynacore::Vector jjpos_;
        dynacore::Vector jtorque_;

        dynacore::Vector torque_command_;
        dynacore::Vector jpos_command_;
        dynacore::Vector jvel_command_;

        NAO_StateEstimator* state_estimator_;
        NAO_StateProvider* sp_;
};

#endif
