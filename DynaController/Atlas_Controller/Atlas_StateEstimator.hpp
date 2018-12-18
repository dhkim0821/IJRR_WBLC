#ifndef STATE_ESTIMATOR_ATLAS
#define STATE_ESTIMATOR_ATLAS

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class Atlas_StateProvider;
class RobotSystem;
class BasicAccumulation;
class Atlas_SensorData;

class Atlas_StateEstimator{
    public:
        Atlas_StateEstimator(RobotSystem* robot);
        ~Atlas_StateEstimator();

        void Initialization(Atlas_SensorData* );
        void Update(Atlas_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        Atlas_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;
};

#endif
