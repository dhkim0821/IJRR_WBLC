#ifndef ANGULAR_VELOCITY_ACCUMULATION_VALKYRIE
#define ANGULAR_VELOCITY_ACCUMULATION_VALKYRIE

#include <Utils/wrap_eigen.hpp>
#include <Filter/filters.hpp>

class BasicAccumulation{
    public:
        BasicAccumulation();
        ~BasicAccumulation(){}

        void EstimatorInitialization(
                const std::vector<double> & acc,
                const std::vector<double> & ang_vel);

        void setSensorData(
                const std::vector<double> & acc,
                const std::vector<double> & ang_vel);

        void getEstimatedState(
                dynacore::Quaternion & ori,
                dynacore::Vect3 & global_ang_vel){
            ori = global_ori_;
            global_ang_vel = global_ang_vel_;
        }

    protected:
        double cutoff_freq_;
        std::vector<filter*> filtered_acc_;

        void _InitIMUOrientationEstimateFromGravity();
        dynacore::Quaternion global_ori_;
        dynacore::Vect3 global_ang_vel_;
};

#endif
