#ifndef BODY_FOOT_KALMAN_FILTER
#define BODY_FOOT_KALMAN_FILTER

#include <Utils/wrap_eigen.hpp>

class Mercury_StateProvider;

class BodyFootObs{
    public:
        dynacore::Vect2 body_led_pos_;
        dynacore::Vect2 rfoot_out_led_pos_;
        dynacore::Vect2 rfoot_in_led_pos_;
        dynacore::Vect2 lfoot_out_led_pos_;
        dynacore::Vect2 lfoot_in_led_pos_;
        dynacore::Vect2 body_led_vel_;
        dynacore::Vector led_visible_;

        BodyFootObs(){}
        ~BodyFootObs(){}
};
class BodyFootInput{
    public:
        dynacore::Vect2 rfoot_out_led_vel_;
        dynacore::Vect2 rfoot_in_led_vel_;
        dynacore::Vect2 lfoot_out_led_vel_;
        dynacore::Vect2 lfoot_in_led_vel_;
        int stance_foot_idx_;

        BodyFootInput(){}
        ~BodyFootInput(){}
};

class BodyFootKalmanFilter{
    public:
        BodyFootKalmanFilter();
        ~BodyFootKalmanFilter();

        void Initialization(void* obs, double height);
        void Estimation(void* obs_input, void * pred_input);

        void getBodyPos(dynacore::Vect3 & body_pos);
        void getFootPos(dynacore::Vect3 & rfoot_pos, dynacore::Vect3 & lfoot_pos);

    protected:
        void _Predict(void* );
        void _Update();
        // State: body x, y
        // rfoot out x, y
        // rfoot in x, y
        // lfoot out x, y
        // lfoot in x, y
        // body vel x, y
        static constexpr int num_state = 12;
        static constexpr int num_obs = 12;
        static constexpr int num_led = 5;
        
        double dt_;
        double body_height_;
        double omega_;

        int stance_foot_state_idx_;
        dynacore::Vect2 stance_loc_;

        // body (x, y) rfoot in/ out(x, y) lfoot in/out (x, y)
        // body vel (x, y)
        dynacore::Vector state_;
         // body (x, y) rfoot in/ out(x, y) lfoot in/out (x, y)
        // body vel (x, y)
        dynacore::Vector state_pred_;
        dynacore::Vector obs_;

        dynacore::Matrix P_;
        
        dynacore::Matrix F_;
        dynacore::Matrix Q_; // prediction covariance
        dynacore::Matrix H_;
        dynacore::Matrix R_; // observation covariance

        Mercury_StateProvider* sp_;
};
#endif
