#ifndef BODY_FOOT_POSITON_ESTIMATOR
#define BODY_FOOT_POSITON_ESTIMATOR

#include <Utils/wrap_eigen.hpp>
#include <Filter/filters.hpp>

class MoCapManager;
class RobotSystem;
class Mercury_StateProvider;
class BodyFootKalmanFilter;
class BodyFootObs;
class BodyFootInput;

class BodyFootPosEstimator{
    public:
        BodyFootPosEstimator(const RobotSystem*);
        ~BodyFootPosEstimator();

        void Initialization(const dynacore::Quaternion & body_ori);
        void Update();

        void getMoCapBodyOri(dynacore::Quaternion & quat);
        void getMoCapBodyVel(dynacore::Vect3 & body_vel);
        void getMoCapBodyPos(const dynacore::Quaternion& body_ori, 
                dynacore::Vect3 & local_body_pos);

    protected:
        MoCapManager* mocap_manager_;
        Mercury_StateProvider* sp_;

        static constexpr int idx_rfoot_out = 6;
        static constexpr int idx_rfoot_in = 7;
        static constexpr int idx_lfoot_out = 11;
        static constexpr int idx_lfoot_in = 12;

        BodyFootKalmanFilter* body_foot_kalman_filter_;
        BodyFootObs* kalman_obs_;
        BodyFootInput* kalman_input_;

        void _KalmanFilterOberservationSetup();
        void _KalmanFilterPredictionInputSetup();

        std::vector<filter*> vel_filter_;
        dynacore::Vect3 body_led_vel_;

        const RobotSystem* robot_sys_;
};

#endif
