#ifndef SR_UPSTAIR_TERRAIN
#define SR_UPSTAIR_TERRAIN

#include "srBlockTerrain.h"

class srUpstairTerrain: public srBlockTerrain{
public:
  srUpstairTerrain(int num_stairs, const Vec3 &location);
  virtual ~srUpstairTerrain();
protected:

};

#endif
