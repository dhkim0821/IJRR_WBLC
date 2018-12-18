#include "ExternalControl.hpp"
#include <Configuration.h>
#include <string>
#include <Utils/comm_udp.hpp>
#include <stdio.h>

int main(int argc, char ** argv){

    int socket;
    ExtCtrl::Location des_loc;

    des_loc.x = 0.;
    des_loc.y = 0.;
    double step_size(0.1);
    char input[1];
    while(true){
        printf("Type input (w - forward, s - backward, a-left, d-right): \n");
        scanf("%s", input);
        if(input[0] == 'w') des_loc.x += step_size;
        else if(input[0] == 's') des_loc.x -= step_size;
        else if(input[0] == 'a') des_loc.y += step_size;
        else if(input[0] == 'd') des_loc.y -= step_size;
        else printf("wrong input\n");
    
        printf("current des location (x, y): %f, %f\n", des_loc.x, des_loc.y);

        COMM::send_data(socket, PORT_EXT_CTRL, 
                &des_loc, sizeof(ExtCtrl::Location), IP_ADDR_MYSELF);
    }
}
