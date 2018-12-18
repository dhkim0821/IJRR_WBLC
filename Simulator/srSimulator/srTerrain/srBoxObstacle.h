#ifndef SR_BOX_OBSTACLE
#define SR_BOX_OBSTACLE

#include "srBlockTerrain.h"
#include "Config_Space_Definition.h"
#include "Utils/utilities.hpp"

class srBoxObstacle: public srBlockTerrain{
public:
  srBoxObstacle(sejong::Vect3 init_pos, sejong::Vect3 lwh_);

  virtual ~srBoxObstacle();
protected:

};

#endif
