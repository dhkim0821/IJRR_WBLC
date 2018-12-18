#ifndef DATA_MANAGER
#define DATA_MANAGER

#include <Utils/dynacore_pThread.hpp>
#include <Addition/Data_Manager/data_protocol.h>
#include <vector>
#include <Utils/wrap_eigen.hpp>
#include <string>

enum DATA_Type{
    DOUBLE,
    INT,
    VECT2,
    VECT3,
    VECT4,
    QUATERNION,
    DYN_VEC
};

class DataManager: public dynacore_pThread{
public:
    static DataManager* GetDataManager();
    virtual ~DataManager();

    void RegisterData(const void* data_addr, DATA_Type dtype, const std::string & dname, int num_array = 1);

    virtual void run();
    void SaveDataFromValues(double * data, int st_idx, int data_idx);

private:
    void _ShowSendingMessage(const DATA_Protocol::DATA_SETUP & data_setup, const double * data);

    DataManager();
    int num_data_;
    int tot_num_array_data_;
    std::vector<const void*> data_addr_vec_;
    std::vector<DATA_Type> data_type_vec_;
    std::vector<int> data_array_size_;
    std::vector<std::string> data_name_vec_;

    int socket1_, socket2_;
};

#endif
