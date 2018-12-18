#ifndef TERRAIN_
#define TERRAIN_

#include "srDyn/srSpace.h"
#include <vector>

#define NUM_STRUCTURE 1

class Terrain : public srSystem{
public:
    Terrain();
    virtual ~Terrain(){}

public:
    srLink link_[NUM_STRUCTURE];
    srCollision collision_[NUM_STRUCTURE];
};


#endif
