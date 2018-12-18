# Whole-Body Control

## Task
1. The following virtual functions must be implemented in child classes. 

````
// Update op_cmd_
virtual bool _UpdateCommand(void* pos_des,
                            const sejong::Vector & vel_des,
                            const sejong::Vector & acc_des) = 0;
// Update Jt_
virtual bool _UpdateTaskJacobian() = 0;

// Update JtDotQdot_
virtual bool _UpdateTaskJDotQdot() = 0;

````
