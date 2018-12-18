#ifndef VAL_MODEL_DEF_H
#define VAL_MODEL_DEF_H

#include <string>
#include <iostream>
#include <stdio.h>

class SR_ValkyrieID{
public:
    enum JointID{
        VIRTUAL_X =0 ,
        VIRTUAL_Y ,
        VIRTUAL_Z ,
        VIRTUAL_Rx ,
        VIRTUAL_Ry ,
        VIRTUAL_Rz  
    };
    
    enum LinkID{
        SIM_Base,
        SIM_L_Y ,
        SIM_L_Z ,
        SIM_L_Rz ,
        SIM_L_Ry ,
        SIM_L_Rx 
    };
    enum WJointID
    {
        WJ_END = 0
    };
};

#endif
