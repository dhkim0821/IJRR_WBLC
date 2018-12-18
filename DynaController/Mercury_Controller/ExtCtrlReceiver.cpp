#include "ExtCtrlReceiver.hpp"
#include "Mercury_StateProvider.hpp"
#include <Utils/comm_udp.hpp>
#include <Configuration.h>

ExtCtrlReceiver::ExtCtrlReceiver():socket_(0){
    sp_ = Mercury_StateProvider::getStateProvider();
}
ExtCtrlReceiver::~ExtCtrlReceiver(){  }

void ExtCtrlReceiver::run(){

    while (true){
        COMM::receive_data(socket_, PORT_EXT_CTRL, 
                &des_loc_, sizeof(ExtCtrl::Location), IP_ADDR_MYSELF);

        sp_->des_location_[0] = des_loc_.x;
        sp_->des_location_[1] = des_loc_.y;
        printf("Current Desired Location (x, y): %f, %f\n", 
                sp_->des_location_[0],
                sp_->des_location_[1]);
    }
}

