#ifndef LED_POS_ANNOUNCER
#define LED_POS_ANNOUNCER

#include "Utils/dynacore_pThread.hpp"
#include <vector>

#define NUM_MARKERS 13
//#define MOCAP_DATA_PORT 51128

//typedef struct{
  //int visible[NUM_MARKERS];
  //double data[NUM_MARKERS*3];
//}mercury_message;


class Mercury_Dyn_environment;

class LED_Position_Announcer: public dynacore_pThread{
public:

    LED_Position_Announcer(Mercury_Dyn_environment* );
    virtual ~LED_Position_Announcer(void){}

    virtual void run(void);

protected:
    int count_;
    int turn_off_count_;
    std::vector<int> led_turn_off_st_count_;
    std::vector<int> led_turn_off_end_count_;
    int socket_;
    std::vector<int> led_link_idx_list_;
    Mercury_Dyn_environment* dyn_env_;
};

#endif
