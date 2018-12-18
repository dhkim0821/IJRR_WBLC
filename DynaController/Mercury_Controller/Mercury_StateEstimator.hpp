#ifndef STATE_ESTIMATOR_MERCURY
#define STATE_ESTIMATOR_MERCURY

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class Mercury_StateProvider;
class RobotSystem;
class filter;
class OriEstimator;
class BodyFootPosEstimator;
class Mercury_SensorData;
class SimpleAverageEstimator;

class Mercury_StateEstimator{
    public:
        Mercury_StateEstimator(RobotSystem* robot);
        ~Mercury_StateEstimator();

        void Initialization(Mercury_SensorData* );
        void Update(Mercury_SensorData* );
        void setFloatingBase(int base_cond){ base_cond_ = base_cond; }
        void setJPosModelUpdate(bool b_enable){ b_jpos_model_update_ = b_enable; }

        bool b_using_jpos_;

    protected:
        void _JointUpdate(Mercury_SensorData* data);
        void _ConfigurationAndModelUpdate();
        void _FootContactUpdate(Mercury_SensorData* data);

        bool b_jpos_model_update_;
        int base_cond_;
        double initial_height_;
        int fixed_foot_;

        dynacore::Quaternion body_ori_;
        dynacore::Vect3 body_ang_vel_;

        dynacore::Vect3 foot_pos_;
        Mercury_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        dynacore::Vector jjpos_config_;
        dynacore::Vector jjvel_qdot_;

        OriEstimator* ori_est_;
        BodyFootPosEstimator* body_foot_est_;
        SimpleAverageEstimator* vel_est_;
        SimpleAverageEstimator* mocap_vel_est_;

        void _RBDL_TEST();
        void _BasicTest();
        void _ProjectionTest();
        void _pseudoInv(const dynacore::Matrix & J, dynacore::Matrix & J_pinv);

};

#endif
