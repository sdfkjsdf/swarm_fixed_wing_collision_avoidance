#ifndef DYNAMICLINEARIZATION_H
#define DYNAMICLINEARIZATION_H
#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include<matrix/Matrix.hpp>


/*
 해당 코드는  "Zero-Order Control Barrier Functions for Sampled-Data Systems with State and Input Dependent Safety Constraints" 을 바탕으로 함

*/




class DynamicsLinearization
{

   public:
    DynamicsLinearization(const FixedwingSpec_t& spec, const double dt);
    ~DynamicsLinearization()=default;

    /*여기서는 지금 dx/dt=f(x)+g(x)u 에서의 f(x),g(x)을 직접 구하는 형태임*/

        void calculate_fx_gx_Jf_x(const matrix::Vector<double,11>& state);


    

    /*calculate_C 은 지금 C := f(xk) − {∂f/∂x(xk)}xk 을 구하는 함수 !! 여기서 반드시 먼저 calculate_Jf_x을 통해서 m_Jf_x을 받아야 함  그리고 지금 f(x) 는  모듈 내부에서 구해야 함  */
        void calculate_C(const matrix::Vector<double, 11 >& state,const matrix::Matrix<double,11,11>& m_Jf_x);


   /* calculate_Ad_Bd 는  AD := eAT, BD := 적분( eA(T−τ)dτ )을 의미함 !! 여기서 반드시 먼저 calculate_Jf_x을 통해서 m_Jf_x을 받아야 함 */
        void calculate_Ad_Bd();

    matrix::Matrix<double,11,11> get_Ad()
        {
            return m_Ad;
        };

    
    inline matrix::Matrix<double,11,4> get_BdB()
        {
            return m_Bd*m_g_x;
        };

    inline matrix::Vector<double,11> get_BdC()
        {
            return m_Bd*m_C;
        };



  private:
     FixedwingSpec_t m_spec;
     double m_dt;
     matrix::Vector<double,11>m_f_x{};
     matrix::Matrix<double,11,4> m_g_x{};
     matrix::Matrix<double,11,11> m_Jf_x{};
     matrix::Matrix<double,11,11> m_Ad{};
     matrix::Matrix<double,11,11> m_Bd{};
     matrix::Vector<double,11> m_C{};



};


#endif