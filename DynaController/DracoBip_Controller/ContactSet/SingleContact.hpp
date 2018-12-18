#ifndef DRACO_BIPED_SINGLE_CONTACT
#define DRACO_BIPED_SINGLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class DracoBip_StateProvider;

class SingleContact: public WBDC_ContactSpec{
    public:
        SingleContact(const RobotSystem* robot, int contact_pt);
        virtual ~SingleContact();

        void setMaxFz(double max_fz){ max_Fz_ = max_fz; }

    protected:
        double max_Fz_;

        virtual bool _UpdateJc();
        virtual bool _UpdateJcDotQdot();
        virtual bool _UpdateUf();
        virtual bool _UpdateInequalityVector();

        const RobotSystem* robot_sys_;
        DracoBip_StateProvider* sp_;

        int contact_pt_;
};

#endif
