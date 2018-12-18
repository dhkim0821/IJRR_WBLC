#ifndef WBC_TASK
#define WBC_TASK

#include <Utils/wrap_eigen.hpp>

class Task{
    public:
        Task(int dim):b_set_task_(false), dim_task_(dim), op_cmd_(dim){}
        virtual ~Task(){}

        void getCommand(dynacore::Vector & op_cmd){  op_cmd = op_cmd_; }
        void getTaskJacobian(dynacore::Matrix & Jt){ Jt = Jt_; }
        void getTaskJacobianDotQdot(dynacore::Vector & JtDotQdot) {
            JtDotQdot = JtDotQdot_;
        }

        bool UpdateTask(void* pos_des, const dynacore::Vector & vel_des, const dynacore::Vector & acc_des){
            _UpdateTaskJacobian();
            _UpdateTaskJDotQdot();
            _UpdateCommand(pos_des, vel_des, acc_des);
            _AdditionalUpdate();
            b_set_task_ = true;
            return true;
        }

        bool IsTaskSet(){ return b_set_task_; }
        int getDim(){ return dim_task_; }
        void UnsetTask(){ b_set_task_ = false; }

    protected:
        // Update op_cmd_
        virtual bool _UpdateCommand(void* pos_des,
                const dynacore::Vector & vel_des,
                const dynacore::Vector & acc_des) = 0;
        // Update Jt_
        virtual bool _UpdateTaskJacobian() = 0;
        // Update JtDotQdot_
        virtual bool _UpdateTaskJDotQdot() = 0;
        // Additional Update (defined in child classes)
        virtual bool _AdditionalUpdate() = 0;

        dynacore::Vector op_cmd_;
        dynacore::Vector JtDotQdot_;
        dynacore::Matrix Jt_;

        bool b_set_task_;
        int dim_task_;
};

#endif
