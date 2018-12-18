#include "LED_Position_Announcer.h"
#include <utils/comm_udp.h>
#include "Dyn_environment.h"


LED_Position_Announcer::LED_Position_Announcer(Dyn_environment* dyn_env):socket_(0){
    dyn_env_ = dyn_env;
}

void LED_Position_Announcer::run(){
    printf("[LED Position Announcer] Start \n");
    led_data_.index = 0;
    led_data_.validBits = 0x001FF; // 1 x 17 F: 4 of 1
    while(true){
        for (int i(0); i<5; ++i){
            led_data_.x[i] = -dyn_env_->m_Hume->LED_[i+3].GetMassCenter()[1]*1000.0;
            led_data_.y[i] = dyn_env_->m_Hume->LED_[i+3].GetMassCenter()[2]* 1000.0;
            led_data_.z[i] = -dyn_env_->m_Hume->LED_[i+3].GetMassCenter()[0]*1000.0;
        }
        led_data_.x[5] = -dyn_env_->m_Hume->LED_[9].GetMassCenter()[1]*1000.0;
        led_data_.y[5] =  dyn_env_->m_Hume->LED_[9].GetMassCenter()[2]*1000.0;
        led_data_.z[5] = -dyn_env_->m_Hume->LED_[9].GetMassCenter()[0]*1000.0;

        led_data_.x[6] = -dyn_env_->m_Hume->LED_[11].GetMassCenter()[1]*1000.0;
        led_data_.y[6] =  dyn_env_->m_Hume->LED_[11].GetMassCenter()[2]*1000.0;
        led_data_.z[6] = -dyn_env_->m_Hume->LED_[11].GetMassCenter()[0]*1000.0;

        led_data_.x[7] = -dyn_env_->m_Hume->LED_[14].GetMassCenter()[1]*1000.0;
        led_data_.y[7] =  dyn_env_->m_Hume->LED_[14].GetMassCenter()[2]*1000.0;
        led_data_.z[7] = -dyn_env_->m_Hume->LED_[14].GetMassCenter()[0]*1000.0;

        led_data_.x[8] = -dyn_env_->m_Hume->LED_[16].GetMassCenter()[1]*1000.0;
        led_data_.y[8] =  dyn_env_->m_Hume->LED_[16].GetMassCenter()[2]*1000.0;
        led_data_.z[8] = -dyn_env_->m_Hume->LED_[16].GetMassCenter()[0]*1000.0;

        COMM::send_data(socket_, POS_PORT, &led_data_, sizeof(message), IP_ADDR_MYSELF);
        ++led_data_.index;
        usleep(1000);
    }
}

