#ifndef STATE_ESTIMATOR_DRACO_BIPED
#define STATE_ESTIMATOR_DRACO_BIPED

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class DracoBip_StateProvider;
class RobotSystem;
class BasicAccumulation;
class DracoBip_SensorData;
class filter;
class BodyEstimator;

class DracoBip_StateEstimator{
    public:
        DracoBip_StateEstimator(RobotSystem* robot);
        ~DracoBip_StateEstimator();

        void Initialization(DracoBip_SensorData* );
        void Update(DracoBip_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        DracoBip_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;
        BodyEstimator* body_est_;
        filter* mocap_x_vel_est_;
        filter* mocap_y_vel_est_;

        void _RBDL_TEST();
};

#endif
