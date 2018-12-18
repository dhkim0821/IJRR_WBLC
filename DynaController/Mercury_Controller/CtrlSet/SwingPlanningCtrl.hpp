#ifndef ABSTRACT_CLASS_SWING_LEG_CONTROL
#define ABSTRACT_CLASS_SWING_LEG_CONTROL

#include <Controller.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <Mercury_Controller/StateEstimator/LIPM_KalmanFilter.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>

class Mercury_StateProvider;
class Planner;
class WBLC_ContactSpec;
class WBLC;
class WBLC_ExtraData;

class SwingPlanningCtrl:public Controller{
    public:
        SwingPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner):
            Controller(robot),
            swing_foot_(swing_foot),
            num_planning_(0),
            planner_(planner),
            planning_frequency_(0.),
            replan_moment_(0.),
            ctrl_start_time_(0.),
            half_swing_time_(0.15),
            b_contact_switch_check_(false){   
                curr_foot_pos_des_.setZero();
                curr_foot_vel_des_.setZero();
                curr_foot_acc_des_.setZero();
                com_estimator_ = new LIPM_KalmanFilter();
                sp_ = Mercury_StateProvider::getStateProvider();
            }

        virtual ~SwingPlanningCtrl(){
        }

        void setPlanningFrequency(double freq){  planning_frequency_ = freq; }
        void setSwingTime(double swing_time){ 
            end_time_ = swing_time; 
            half_swing_time_ = end_time_/2.;
        }
        void setDoubleStanceRatio(double ratio){  double_stance_ratio_ = ratio;}
        void setTransitionPhaseRatio(double ratio){  transition_phase_ratio_ = ratio;}

        void notifyTransitionTime(double time){  transition_time_ = time; }
        void notifyStanceTime(double time){  stance_time_ = time; }

        void setPrimeTimeX(double t_p_x){ t_prime_x_ = t_p_x; }
        void setPrimeTimeY(double t_p_y){ t_prime_y_ = t_p_y; }
        void setStanceHeight(double height) {
            des_body_height_ = height;
            b_set_height_target_ = true;
            // CoM estimator
            com_estimator_->h_ = des_body_height_;
        }
        void setContactSwitchCheck(bool switch_check){ b_contact_switch_check_ = switch_check; }

        dynacore::Vect3 curr_foot_pos_des_;
        dynacore::Vect3 curr_foot_vel_des_;
        dynacore::Vect3 curr_foot_acc_des_;

    protected:
        void _CoMEstiamtorUpdate(){
            dynacore::Vect3 com_pos, com_vel;
            robot_sys_->getCoMPosition(com_pos);
            robot_sys_->getCoMVelocity(com_vel);
            dynacore::Vector input_state(4);
            input_state[0] = com_pos[0];   input_state[1] = com_pos[1];
            input_state[2] = com_vel[0];   input_state[3] = com_vel[1];
            com_estimator_->InputData(input_state);
            com_estimator_->Output(sp_->estimated_com_state_);
        }
        bool b_contact_switch_check_;
        bool b_set_height_target_;
        double des_body_height_;

        double double_stance_ratio_;
        double transition_phase_ratio_;
        double replan_moment_;

        int swing_foot_;
        double swing_height_;
        dynacore::Vect3 default_target_loc_;

        double planning_frequency_;
        int num_planning_;
        double t_prime_x_;
        double t_prime_y_;

        WBLC_ContactSpec* single_contact_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;
        Planner* planner_;

        CoMStateEstimator* com_estimator_;
        Mercury_StateProvider* sp_;
    
        // Timing parameters
        double end_time_;
        double half_swing_time_;
        double transition_time_;
        double stance_time_;
        double ctrl_start_time_;
};

#endif
