#ifndef WHOLE_BODY_Locomotion_CONTROL_CONTACT_SPEC
#define WHOLE_BODY_Locomotion_CONTROL_CONTACT_SPEC

#include <ContactSpec.hpp>

class WBLC_ContactSpec:public ContactSpec{
    public:
        WBLC_ContactSpec(int dim):ContactSpec(dim) { idx_Fz_ = dim -1; }
        virtual ~WBLC_ContactSpec(){}

        virtual int getDimRFConstratint() { return Uf_.rows(); }
        void getRFConstraintMtx(dynacore::Matrix & Uf){ Uf = Uf_; }
        void getRFConstraintVec(dynacore::Vector & ieq_vec){ ieq_vec = ieq_vec_; }

        int getFzIndex(){ return idx_Fz_; }
    protected:
        int idx_Fz_;
        virtual bool _AdditionalUpdate(){
            _UpdateUf();
            _UpdateInequalityVector();
            return true;
        }
        virtual bool _UpdateUf() = 0;
        virtual bool _UpdateInequalityVector() = 0;

        dynacore::Matrix Uf_;
        dynacore::Vector ieq_vec_;
};

#endif
