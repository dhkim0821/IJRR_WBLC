#ifndef SR_PLANE_TERRAIN
#define SR_PLANE_TERRAIN

#include "srBlockTerrain.h"

class srPlane: public srBlockTerrain{
 public:
  srPlane(double slope, const Vec3 &location);
  virtual ~srPlane();
 protected:

};

#endif
