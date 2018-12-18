#ifndef LED_POS_ANNOUNCER
#define LED_POS_ANNOUNCER

#include "utils/Sejong_Thread.h"
#include <ControlSystem/Hume_Controller/MoCap_udp.h>


class Dyn_environment;

class LED_Position_Announcer: public Sejong_Thread{
public:
    
    LED_Position_Announcer(Dyn_environment* );
    virtual ~LED_Position_Announcer(void){}

    virtual void run(void);

protected:
    int socket_;
    Dyn_environment* dyn_env_;
    message led_data_;
};

#endif
