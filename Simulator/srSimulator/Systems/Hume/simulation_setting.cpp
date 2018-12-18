#include "simulation_setting.h"
#include <ControlSystem/Hume_Controller/HumeSystem.h>


simulation_setting::simulation_setting():display_landing_loc_(false),
                                         no_stabilizing_(false),
                                         hanging_(false),
                                         initial_pos_(0)
                                         
{
    simulation_setting_file.open( THIS_COM"test_config/simulation_setting.txt");

    string output;
    if(simulation_setting_file.is_open()){

        while(!simulation_setting_file.eof()){
            simulation_setting_file >> output;

            if(output == "Landing_Location_Display:"){
                simulation_setting_file >> output;
                if( output == "true") { display_landing_loc_ = true; }
            }

            if(output == "No_Stabilizing:"){
                simulation_setting_file >> output;
                if( output == "true") { no_stabilizing_ = true; }
            }
            if(output == "Hanging_in_air:"){
                simulation_setting_file >> output;
                if( output == "true") { hanging_ = true; }
            }
            if(output == "Initial_Pose:"){
                simulation_setting_file >> output;
                initial_pos_ = std::atoi(output.c_str());
            }
            if(output == "Holding_count:"){
                simulation_setting_file >> output;
                holding_count_ = std::atoi(output.c_str());
            }
        }
    }
}

simulation_setting::~simulation_setting(){}
