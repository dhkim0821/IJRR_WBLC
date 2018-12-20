#ifndef BODY_CTRL_VALKYRIE
#define BODY_CTRL_VALKYRIE

#include <Controller.hpp>

class Valkyrie_StateProvider;
class RobotSystem;
class WBLC_ContactSpec;
class WBLC;
class WBLC_ExtraData;
class KinWBC;

class BodyCtrl: public Controller{
    public:
        BodyCtrl(RobotSystem* );
        virtual ~BodyCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double time) { end_time_ = time; }
        void setStanceHeight(double height){ 
            target_body_height_ = height;
            b_set_height_target_ = true;
         }

    protected:
        dynacore::Vector Kp_, Kd_;
        dynacore::Vector des_jpos_; 
        dynacore::Vector des_jvel_; 
        dynacore::Vector des_jacc_;

        dynacore::Vector jpos_ini_;
        bool b_set_height_target_;
        double end_time_;
        int dim_contact_;

        std::vector<int> selected_jidx_;
        Task* head_ori_task_;
        Task* head_joint_task_;

        Task* total_joint_task_;
        Task* body_pos_task_; //pelvis
        Task* body_ori_task_;
        Task* torso_ori_task_;

        Task* lhand_pos_task_;
        Task* lhand_ori_task_;
        Task* selected_joint_task_;
        KinWBC* kin_wbc_;
        WBLC_ContactSpec* rfoot_contact_;
        WBLC_ContactSpec* lfoot_contact_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        double target_body_height_;
        double ini_body_height_;

        dynacore::Quaternion ini_lhand_ori_;
        dynacore::Vect3 ini_lhand_pos_;
        dynacore::Vect3 ini_body_pos_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Valkyrie_StateProvider* sp_;
};

#endif
