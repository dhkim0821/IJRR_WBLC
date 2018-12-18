#ifndef STATE_ESTIMATOR_VALKYRIE
#define STATE_ESTIMATOR_VALKYRIE

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class Valkyrie_StateProvider;
class RobotSystem;
class BasicAccumulation;
class Valkyrie_SensorData;

class Valkyrie_StateEstimator{
    public:
        Valkyrie_StateEstimator(RobotSystem* robot);
        ~Valkyrie_StateEstimator();

        void Initialization(Valkyrie_SensorData* );
        void Update(Valkyrie_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        Valkyrie_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;

        void _RBDL_TEST();
        void _BasicTest();
        void _ProjectionTest();
        void _pseudoInv(const dynacore::Matrix & J, dynacore::Matrix & J_pinv);

};

#endif
