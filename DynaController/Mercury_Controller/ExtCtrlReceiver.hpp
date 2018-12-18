#ifndef ExtCtrlReceiver_HPP
#define ExtCtrlReceiver_HPP

#include <Addition/ExternalControl/ExternalControl.hpp>
#include <Utils/dynacore_pThread.hpp>

class Mercury_StateProvider;

class ExtCtrlReceiver: public dynacore_pThread{
    public:
        ExtCtrlReceiver();
        virtual ~ExtCtrlReceiver();

        virtual void run();

    private:
        ExtCtrl::Location des_loc_;
        Mercury_StateProvider* sp_;
        int socket_;
};
#endif
