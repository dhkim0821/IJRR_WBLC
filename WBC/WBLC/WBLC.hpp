#ifndef WHOLE_BODY_LOCOMOTION_CONTROL_H
#define WHOLE_BODY_LOCOMOTION_CONTROL_H

#include <WBC.hpp>
#include <Utils/utilities.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>
#include <WBLC/WBLC_ContactSpec.hpp>

class WBLC_ExtraData{
    public:
        // Output
        dynacore::Vector opt_result_;
        dynacore::Vector qddot_;
        dynacore::Vector Fr_;

        // Input
        dynacore::Vector W_qddot_;
        dynacore::Vector W_rf_;
        dynacore::Vector W_xddot_;

        dynacore::Vector tau_min_;
        dynacore::Vector tau_max_;

        WBLC_ExtraData(){}
        ~WBLC_ExtraData(){}
};

class WBLC: public WBC{
    public:
        WBLC(const std::vector<bool> & act_list, const dynacore::Matrix* Jci = NULL);
        virtual ~WBLC(){}

        virtual void UpdateSetting(const dynacore::Matrix & A,
                const dynacore::Matrix & Ainv,
                const dynacore::Vector & cori,
                const dynacore::Vector & grav,
                void* extra_setting = NULL);

        void MakeWBLC_Torque(const dynacore::Vector & des_jacc_cmd,
                const std::vector<ContactSpec*> & contact_list,
                dynacore::Vector & cmd,
                void* extra_input = NULL); 
        
        virtual void MakeTorque(
                const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                dynacore::Vector & cmd,
                void* extra_input = NULL);

       

    private:
        std::vector<int> act_list_;

        void _OptimizationPreparation(
                const dynacore::Matrix & Aeq, 
                const dynacore::Vector & beq,
                const dynacore::Matrix & Cieq,
                const dynacore::Vector & dieq);


        void _GetSolution(dynacore::Vector & cmd);
        void _OptimizationPreparation();

        int dim_opt_;
        int dim_eq_cstr_; // equality constraints
        int dim_ieq_cstr_; // inequality constraints
        int dim_first_task_; // first task dimension
        WBLC_ExtraData* data_;

        GolDIdnani::GVect<double> z;
        // Cost
        GolDIdnani::GMatr<double> G;
        GolDIdnani::GVect<double> g0;

        // Equality
        GolDIdnani::GMatr<double> CE;
        GolDIdnani::GVect<double> ce0;

        // Inequality
        GolDIdnani::GMatr<double> CI;
        GolDIdnani::GVect<double> ci0;

        int dim_rf_;
        int dim_relaxed_task_;
        int dim_cam_;
        int dim_rf_cstr_;
        
        // BuildContactMtxVect builds the followings:
        void _BuildContactMtxVect(const std::vector<ContactSpec*> & contact_list);
        dynacore::Matrix Uf_;
        dynacore::Vector Fr_ieq_;
        dynacore::Matrix Jc_;
        dynacore::Vector JcDotQdot_;
        
        // Setup the followings:
        void _Build_Equality_Constraint();
        dynacore::Matrix Aeq_;
        dynacore::Vector beq_;

        void _Build_Inequality_Constraint();
        dynacore::Matrix Cieq_;
        dynacore::Vector dieq_;

        dynacore::Vector qddot_;

        dynacore::Matrix Sf_; //floating base
        void _PrintDebug(double i) {
            //printf("[WBLC] %f \n", i);
        }
};

#endif
