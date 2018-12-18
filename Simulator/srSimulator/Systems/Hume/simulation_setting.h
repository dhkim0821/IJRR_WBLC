#ifndef SIMULATION_SETTING
#define SIMULATION_SETTING

#include <fstream>

using namespace std;

class simulation_setting{
public:
    simulation_setting();
    ~simulation_setting();

public:
    bool display_landing_loc_;
    bool no_stabilizing_;
    bool hanging_;
    int initial_pos_;
    int holding_count_;
    ifstream simulation_setting_file;
};

#endif 
