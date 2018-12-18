#ifndef LQR_SOLVER
#define LQR_SOLVER

#include "wrap_eigen.hpp"

namespace dynacore{
    void LQR(const Matrix & A,
             const Matrix & B,
             const Matrix & Q,
             const Matrix & R,
             Matrix & S,
             bool descrete = false);
}

#endif
