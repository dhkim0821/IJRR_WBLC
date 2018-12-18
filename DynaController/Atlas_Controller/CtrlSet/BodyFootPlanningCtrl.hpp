#ifndef BODY_FOOT_WALKING_CONTROL_ATLAS
#define BODY_FOOT_WALKING_CONTROL_ATLAS

#include "SwingPlanningCtrl.hpp"
#include <Utils/minjerk_one_dim.hpp>
#include <Utils/BSplineBasic.h>


class BodyFootPlanningCtrl:public SwingPlanningCtrl{
   public:
        BodyFootPlanningCtrl(const RobotSystem* robot, 
                int swing_foot, Planner* planner);
        virtual ~BodyFootPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit(){ sp_->des_jpos_prev_ = des_jpos_; }
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
    protected:
        double waiting_time_limit_;
        double ini_base_height_;
        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing

        dynacore::Vect3 default_target_loc_;
        dynacore::Vect3 initial_target_loc_;
        
        int dim_contact_;
        WBDC_ContactSpec* rfoot_contact_;
        WBDC_ContactSpec* lfoot_contact_;

        void _CheckPlanning();
        void _Replanning(dynacore::Vect3 & target_loc);
        void _contact_setup();
        void _task_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);
        void _SetMinJerkOffset(const dynacore::Vect3 & offset);
        void _SetBspline(
            const dynacore::Vect3 & st_pos, 
            const dynacore::Vect3 & des_pos);

        void _GetSinusoidalSwingTrajectory();
        void _GetBsplineSwingTrajectory();
        void _foot_pos_task_setup();
        std::vector<ContactSpec*> kin_wbc_contact_list_;

        Task* body_pos_task_;
        Task* body_ori_task_;
        Task* torso_ori_task_;
        Task* foot_pos_task_;
        Task* foot_ori_task_;
        Task* total_joint_task_;

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        dynacore::Vector Kp_;
        dynacore::Vector Kd_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect2 body_pt_offset_;
        
        dynacore::Vector ini_config_;

        std::vector<double> foot_landing_offset_;

        std::vector<MinJerk_OneDimension*> min_jerk_offset_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;
};

#endif
