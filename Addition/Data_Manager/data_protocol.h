#ifndef DATA_PROTOCOL
#define DATA_PROTOCOL

#include <stdint.h>

#define IP_ADDR_MOCAP "192.168.1.149"
#define IP_ADDR_MAC    "192.168.1.131"
#define IP_ADDR_DH    "192.168.50.159"

//#define IP_ADDR IP_ADDR_DH
#define IP_ADDR IP_ADDR_MYSELF

#define MAX_NUM_DATA 100

namespace DATA_Protocol{
    typedef struct{
        int num_data;
        int tot_num_array_data;
        char data_name[MAX_NUM_DATA][30];
        int num_array_data[MAX_NUM_DATA];
    }DATA_SETUP;
};

#endif
