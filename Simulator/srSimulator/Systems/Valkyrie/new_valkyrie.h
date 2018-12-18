#ifndef NEW_VALKYRIE
#define NEW_VALKYRIE

#include <srSysGenerator/SystemGenerator.h>

class New_Valkyrie: public SystemGenerator {
 public:
  New_Valkyrie();
  virtual ~New_Valkyrie();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif
