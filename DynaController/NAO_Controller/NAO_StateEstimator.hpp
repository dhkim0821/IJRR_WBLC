#ifndef STATE_ESTIMATOR_NAO
#define STATE_ESTIMATOR_NAO

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class NAO_StateProvider;
class RobotSystem;
class BasicAccumulation;
class NAO_SensorData;

class NAO_StateEstimator{
    public:
        NAO_StateEstimator(RobotSystem* robot);
        ~NAO_StateEstimator();

        void Initialization(NAO_SensorData* );
        void Update(NAO_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        NAO_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;
};

#endif
