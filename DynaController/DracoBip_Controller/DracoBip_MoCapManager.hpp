#ifndef MOTION_CAPTURE_DATA_MANAGER_DRACO_BIPED
#define MOTION_CAPTURE_DATA_MANAGER_DRACO_BIPED

#include <Utils/dynacore_pThread.hpp>
#include <Utils/wrap_eigen.hpp>
#include <DracoBip_Controller/StateEstimator/BodyEstimator.hpp>
#include <Filter/filters.hpp>

class DracoBip_StateProvider;
class RobotSystem;

////////////////////////////////////////////
// Must to be synchronized with MoCap system
/////////////////////////////////////////////
#define MOCAP_DATA_PORT 51128
//#define IP_ADDRESS "192.168.0.144"
#define IP_ADDRESS "192.168.0.211"
#define NUM_MARKERS 13

typedef struct{
    // float condition[NUM_MARKERS];
    double data[NUM_MARKERS * 3];
}message;
////////////////////////////////////////////
typedef struct{
    int visible[NUM_MARKERS];
    double data[NUM_MARKERS*3];
}dracobip_message;


class DracoBip_MoCapManager: public dynacore_pThread{
    public:
        friend class BodyEstimator;

        DracoBip_MoCapManager(const RobotSystem* );
        virtual ~DracoBip_MoCapManager(){}

        virtual void run(void);

        dynacore::Quaternion body_quat_;
        void CoordinateUpdateCall(){ b_update_call_ = true; }

    protected:
        std::vector<dynacore::Vect3> healthy_led_list_;

        std::vector<filter*> body_led0_filter_;
        std::vector<filter*> body_led1_filter_;
        std::vector<filter*> body_led2_filter_;

        double initialization_duration_;
        dynacore::Vect3 offset_;
        dynacore::Quaternion imu_body_ori_;

        DracoBip_StateProvider * sp_;
        dynacore::Matrix R_coord_;
        bool b_update_call_;

        void _print_message(const dracobip_message & msg);
        void _UpdateLEDPosData(const dracobip_message & msg);
        void _CoordinateUpdate(dracobip_message & msg);
        void _CoordinateChange(dracobip_message & msg);
        dynacore::Matrix _GetOrientation(const dynacore::Vect3 &, 
                const dynacore::Vect3 &, const dynacore::Vect3 &);

        int socket_;
        std::vector<int> marker_cond_;

        dynacore::Vector led_pos_data_;
        dynacore::Vector led_kin_data_;
        dynacore::Vector led_pos_raw_data_;

        int lfoot_idx;
        int rfoot_idx;

        const RobotSystem* robot_sys_;
};

#endif
