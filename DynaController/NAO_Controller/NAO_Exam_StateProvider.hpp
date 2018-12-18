#ifndef NAO_EXAMPLE_STATE_PROVIDER
#define NAO_EXAMPLE_STATE_PROVIDER

#include <Utils/wrap_eigen.hpp>
#include <NAO/NAO_Definition.h>

class NAO_Exam_StateProvider{
    public:
        dynacore::Vector q_;
        dynacore::Vector qdot_;


        static NAO_Exam_StateProvider* getStateProvider(){
            static NAO_Exam_StateProvider sp;
            return & sp;
        }

    private:
        NAO_Exam_StateProvider():q_(nao::num_q), qdot_(nao::num_qdot){}
};
#endif
