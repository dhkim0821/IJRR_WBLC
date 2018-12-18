#ifndef Valkyrie_DYN_MODEL
#define Valkyrie_DYN_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Valkyrie_Dyn_Model{
public:

    Valkyrie_Dyn_Model(RigidBodyDynamics::Model* model);
    ~Valkyrie_Dyn_Model(void);

    bool getMassInertia(dynacore::Matrix & a);
    bool getInverseMassInertia(dynacore::Matrix & ainv);
    bool getGravity(dynacore::Vector &  grav);
    bool getCoriolis(dynacore::Vector & coriolis);

    void UpdateDynamics(const dynacore::Vector & q, const dynacore::Vector & qdot);

protected:
    dynacore::Matrix A_;
    dynacore::Matrix Ainv_;
    dynacore::Vector grav_;
    dynacore::Vector coriolis_;

    RigidBodyDynamics::Model* model_;
};

#endif
