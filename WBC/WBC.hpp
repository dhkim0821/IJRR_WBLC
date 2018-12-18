#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include "Task.hpp"
#include "ContactSpec.hpp"

// Assume first 6 (or 3 in 2D case) joints are for the representation of 
// a floating base. 

class WBC{
    public:
        WBC(const std::vector<bool> & act_list, 
                const dynacore::Matrix * Jc_internal = NULL):
            num_act_joint_(0),
            num_passive_(0),
            b_internal_constraint_(false)
    {
        num_qdot_ = act_list.size();
        for(int i(0); i<num_qdot_; ++i){
            if(act_list[i] == true) ++num_act_joint_;
            else ++num_passive_;
        }
        Sa_ = dynacore::Matrix::Zero(num_act_joint_, num_qdot_);
        Sv_ = dynacore::Matrix::Zero(num_passive_, num_qdot_);

        // Set virtual & actuated selection matrix
        int j(0);
        int k(0);
        for(int i(0); i <num_qdot_; ++i){
            if(act_list[i] == true){
                Sa_(j, i) = 1.;
                ++j;
            }
            else{
                Sv_(k,i) = 1.;
                ++k;
            }
        }

        if(Jc_internal){
            Jci_ = *Jc_internal;
            b_internal_constraint_ = true;
        }

        // dynacore::pretty_print(Sa_, std::cout, "Sa");
        // dynacore::pretty_print(Sv_, std::cout, "Sv");
    }
        virtual ~WBC(){}

        virtual void UpdateSetting(const dynacore::Matrix & A,
                const dynacore::Matrix & Ainv,
                const dynacore::Vector & cori,
                const dynacore::Vector & grav,
                void* extra_setting = NULL) = 0;

        virtual void MakeTorque(const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                dynacore::Vector & cmd,
                void* extra_input = NULL) =0;

    protected:
        // full rank fat matrix only
        void _WeightedInverse(const dynacore::Matrix & J,
                const dynacore::Matrix & Winv,
                dynacore::Matrix & Jinv, double threshold = 0.0001){
            dynacore::Matrix lambda(J* Winv * J.transpose());
            dynacore::Matrix lambda_inv;
            dynacore::pseudoInverse(lambda, threshold, lambda_inv);
            Jinv = Winv * J.transpose() * lambda_inv;
        }

        int num_qdot_;
        int num_act_joint_;
        int num_passive_;

        dynacore::Matrix Sa_; // Actuated joint
        dynacore::Matrix Sv_; // Virtual joint

        dynacore::Matrix A_;
        dynacore::Matrix Ainv_;
        dynacore::Vector cori_;
        dynacore::Vector grav_;

        bool b_updatesetting_;

        bool b_internal_constraint_;
        dynacore::Matrix Jci_; // internal constraint Jacobian
        dynacore::Matrix Nci_;
};

#endif
