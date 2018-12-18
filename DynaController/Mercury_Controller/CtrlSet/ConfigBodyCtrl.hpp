#ifndef CONFIGURATION_BODY_CTRL
#define CONFIGURATION_BODY_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class WBLC_ContactSpec;

class ConfigBodyCtrl: public Controller{
    public:
        ConfigBodyCtrl(RobotSystem* );
        virtual ~ConfigBodyCtrl();

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
        dynacore::Vector jpos_ini_;
        dynacore::Vector jpos_target_;
        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        bool b_set_height_target_;
        int trj_type_;
        int contact_dim_;
        double end_time_;

        Task* base_task_;
        KinWBC* kin_wbc_;
        WBLC_ContactSpec* rfoot_contact_;
        WBLC_ContactSpec* lfoot_contact_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        double target_body_height_;
        double ini_body_height_;
        bool b_jpos_set_;

        void _base_task_setup();
        void _double_contact_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Mercury_StateProvider* sp_;
};

#endif
