#ifndef DRACO_BIPED_FOOT_LINEAR
#define DRACO_BIPED_FOOT_LINEAR

#include <WBLC/WBLC_ContactSpec.hpp>
class RobotSystem;
class DracoBip_StateProvider;

class FootLinear: public WBLC_ContactSpec{
    public:
        FootLinear(const RobotSystem* robot, int contact_pt);
        virtual ~FootLinear();

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
