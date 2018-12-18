#ifndef DOUBLE_CONTACT_TRANSITION_CONFIGURATION_CTRL
#define DOUBLE_CONTACT_TRANSITION_CONFIGURATION_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBLC_ContactSpec;
class KinWBC;
class WBLC;
class WBLC_ExtraData;

class DoubleContactTransCtrl: public Controller{
    public:
        DoubleContactTransCtrl(RobotSystem* );
        virtual ~DoubleContactTransCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double stance_time){ end_time_ = stance_time; }
        void setStanceHeight(double height) {
            des_base_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
        bool b_set_height_target_;
        double des_base_height_;
        double ini_base_height_;
        double max_rf_z_;
        double min_rf_z_;
        int dim_contact_;

        Task* base_task_;
        WBLC_ContactSpec* rfoot_contact_;
        WBLC_ContactSpec* lfoot_contact_;
        KinWBC* kin_wbc_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        dynacore::Vector base_pos_ini_;
        dynacore::Vect3 ini_base_pos_;

        dynacore::Vector ini_jpos_;
        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        dynacore::Vector Kp_;
        dynacore::Vector Kd_;

        double end_time_;
        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);
        double ctrl_start_time_;

        Mercury_StateProvider* sp_;
};

#endif
