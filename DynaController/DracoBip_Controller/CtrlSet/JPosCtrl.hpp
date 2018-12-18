#ifndef DRACO_BIPED_JOINT_POSITION_CTRL
#define DRACO_BIPED_JOINT_POSITION_CTRL

#include <Controller.hpp>

class DracoBip_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC;
class WBDC_ExtraData;


class JPosCtrl: public Controller{
    public:
        JPosCtrl(RobotSystem* );
        virtual ~JPosCtrl();

        virtual void OneStep(void* cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setMovingTime(double time) { end_time_ = time; }
        void setPosture(const std::vector<double> & set_jpos){
            set_jpos_ = set_jpos;
            b_jpos_set_ = true;
        }

        // For sinusoidal trajectory test
        void setAmplitude(const std::vector<double> & amp){ amp_ = amp; }
        void setFrequency(const std::vector<double> & freq){ freq_ = freq; }
        void setPhase(const std::vector<double> & phase){ phase_ = phase; }

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

        bool b_jpos_set_;
        std::vector<double> set_jpos_;

        // For sinusoidal trajectory test
        std::vector<double> amp_;
        std::vector<double> freq_;
        std::vector<double> phase_;

        void _jpos_task_setup();
        void _fixed_body_contact_setup();
        void _jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma);

        double ctrl_start_time_;
        DracoBip_StateProvider* sp_;
};

#endif
