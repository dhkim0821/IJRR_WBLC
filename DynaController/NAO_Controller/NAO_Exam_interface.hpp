#ifndef NAO_EXAMPLE_INTERFACE_H
#define NAO_EXAMPLE_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include <NAO/NAO_Definition.h>
class NAO_Exam_StateProvider;

class NAO_Exam_SensorData{
    public:
        double q[nao::num_q];
        double qdot[nao::num_qdot];
};

class NAO_Exam_interface : public interface{
    public:
        NAO_Exam_interface();
        virtual ~NAO_Exam_interface();

        virtual void GetCommand(void* sensor_data, std::vector<double> & command);

    protected:
        NAO_Exam_StateProvider* sp_;
        bool _Initialization(void * sensor_data);
        dynacore::Vector initial_jpos_;
        dynacore::Vector gamma_;
};

#endif
