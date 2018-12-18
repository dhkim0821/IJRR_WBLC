#ifndef SR_ROOM
#define SR_ROOM

#include "srBlockTerrain.h"
#include "Config_Space_Definition.h"

class srRoom: public srBlockTerrain{
public:
  srRoom(const Vec3 &location);
  virtual ~srRoom();
protected:

};

#endif
