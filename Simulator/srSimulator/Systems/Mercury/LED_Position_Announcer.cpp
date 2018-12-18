#include "LED_Position_Announcer.hpp"
#include "Mercury_Dyn_environment.hpp"
//#include <Addition/Data_Manager/data_protocol.h>
#include <Utils/comm_udp.hpp>
#include <Mercury_Controller/MoCapManager.hpp>

LED_Position_Announcer::LED_Position_Announcer(Mercury_Dyn_environment* dyn_env):
    socket_(0), count_(0){
    dyn_env_ = dyn_env;
    turn_off_count_ = 0;

    //led_turn_off_st_count_.push_back(1000);
    //led_turn_off_end_count_.push_back(1500);

    //led_turn_off_st_count_.push_back(3000);
    //led_turn_off_end_count_.push_back(3300);

    //led_turn_off_st_count_.push_back(4000);
    //led_turn_off_end_count_.push_back(4500);

}

void LED_Position_Announcer::run(){
     printf("[LED Position Announcer] Start \n");
     
     mercury_message mercury_msg;
     led_link_idx_list_.clear();

     // Link idx list 
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("body_led0")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("body_led1")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("body_led2")->second) );

     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("rleg_led0")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("rleg_led1")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("rleg_led2")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("rleg_led3")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("rleg_led4")->second) );

     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("lleg_led0")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("lleg_led1")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("lleg_led2")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("lleg_led3")->second) );
     led_link_idx_list_.push_back( (dyn_env_->m_Mercury->link_idx_map_.find("lleg_led4")->second) );



     while(true){
         for(int j(0); j<NUM_MARKERS; ++j){
             mercury_msg.visible[j] = 1;
             for (int i(0); i<3; ++i){
                 mercury_msg.data[3*j + i] = 
                     dyn_env_->m_Mercury->link_[ led_link_idx_list_[j] ]->GetMassCenter()[i]*1000.0;
             }
         }

         if(led_turn_off_st_count_.size()>0){
         if(count_>led_turn_off_st_count_[turn_off_count_]){
             mercury_msg.visible[7] = 0;
             mercury_msg.visible[12] = 0;
             for(int i(0); i<3; ++i){
                 mercury_msg.data[3*7 + i] = 0.;
                 mercury_msg.data[3*12 + i] = 0.;
             }
         }
         if(count_> led_turn_off_end_count_[turn_off_count_]){
              mercury_msg.visible[7] = 1;
             mercury_msg.visible[12] = 1;
             ++turn_off_count_;
             for(int i(0); i<3; ++i){
                 mercury_msg.data[3*7 + i] =
                     dyn_env_->m_Mercury->link_[ led_link_idx_list_[7] ]->GetMassCenter()[i]*1000.0;
                 mercury_msg.data[3*12 + i] =
                     dyn_env_->m_Mercury->link_[ led_link_idx_list_[12] ]->GetMassCenter()[i]*1000.0;
             }
          }
         }
         COMM::send_data(socket_, MOCAP_DATA_PORT, 
                 &mercury_msg, sizeof(mercury_message), IP_ADDR_MYSELF);
         usleep(2000);
         ++count_;
         // if(count_%100 == 1)  printf("count: %d\n", count_);
     }
}

