#ifndef BALL_THROW
#define BALL_THROW

#include "srDyn/srSpace.h"
#include <vector>

#define NUM_BALL 3

class Ball: public srSystem{
public:
    Ball(double, double, double, double, double, double);
    virtual ~Ball(){}

public:
    srLink ball_;
    srCollision collision_;
};

#endif
