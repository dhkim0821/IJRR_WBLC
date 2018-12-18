#ifndef NAO_EXAMPLE_JOINT_POSITION_CTRL
#define NAO_EXAMPLE_JOINT_POSITION_CTRL

#include <Controller.hpp>
class NAO_Exam_StateProvider;

class JPosCtrl: public Controller{
public:
  JPosCtrl(RobotSystem*);
  virtual ~JPosCtrl();

  virtual void OneStep(dynacore::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

protected:
  NAO_Exam_StateProvider* sp_;
  dynacore::Vector ini_jpos_;
};

#endif
