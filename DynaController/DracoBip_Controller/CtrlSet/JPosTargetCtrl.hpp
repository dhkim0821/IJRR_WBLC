#ifndef JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL_DRACO_BIPED
#define JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL_DRACO_BIPED

#include <Controller.hpp>

class DracoBip_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC;
class WBDC_ExtraData;

class JPosTargetCtrl: public Controller{
    public:
        JPosTargetCtrl(RobotSystem* );
        virtual ~JPosTargetCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setMovingTime(double time) { end_time_ = time; }
        void setTargetPosition(const std::vector<double> & jpos);
    protected:
        double end_time_;

        Task* jpos_task_;
        WBDC_ContactSpec* fixed_body_contact_;
        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;

        dynacore::Vector jpos_ini_;
        dynacore::Vector jpos_target_;
        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;

        void _jpos_task_setup();
        void _fixed_body_contact_setup();
        void _jpos_ctrl_wbdc(dynacore::Vector & gamma);

        DracoBip_StateProvider* sp_;
        double ctrl_start_time_;
};

#endif
