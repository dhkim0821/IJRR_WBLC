#ifndef KINEMATICS_TASK
#define KINEMATICS_TASK

#include <Task.hpp>

class KinTask:public Task{
    public:
        KinTask(int dim):Task(dim),
        pos_err_(dim),
        vel_des_(dim),
        acc_des_(dim) {}

        virtual ~KinTask(){}

        dynacore::Vector pos_err_;
        dynacore::Vector vel_des_;
        dynacore::Vector acc_des_;
};

#endif
