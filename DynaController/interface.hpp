#ifndef INTERFACE_H
#define INTERFACE_H

class Test;
class RobotSystem;

class interface{
public:
  interface():count_(0), running_time_(0.){}
  virtual ~interface(){}

  virtual void GetCommand(void* sensor_data,  void* command) = 0;

protected:
  Test* test_;
  RobotSystem* robot_sys_;

  int count_;
  double running_time_;
};

#endif
