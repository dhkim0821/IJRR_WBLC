#ifndef KINEMATICS_WHOLE_BODY_CONTROL
#define KINEMATICS_WHOLE_BODY_CONTROL

#include <ContactSpec.hpp>
#include "KinTask.hpp"
#include <vector>

class KinWBC{
    public:
        KinWBC(const std::vector<bool> & act_joint);
        ~KinWBC(){}

        bool FindConfiguration(
                const dynacore::Vector & curr_config,
                const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                dynacore::Vector & jpos_cmd,
                dynacore::Vector & jvel_cmd,
                dynacore::Vector & jacc_cmd);

        dynacore::Matrix Ainv_;
    private:
        void _PseudoInverse(const dynacore::Matrix J, dynacore::Matrix & Jinv);
        void _BuildProjectionMatrix(
                const dynacore::Matrix & J,
                dynacore::Matrix & N);

        double threshold_;
        int num_qdot_;
        int num_act_joint_;
        std::vector<int> act_jidx_;
        dynacore::Matrix I_mtx;
};
#endif

