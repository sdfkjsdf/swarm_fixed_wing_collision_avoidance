#include "FixedwingDynamics.h"
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>
#include <matrix/math.hpp>
#include <cmath>

FixedwingDynamics ::FixedwingDynamics(const FixedwingSpec_t& spec)
    :m_spec(spec)

{ 
};

 matrix::Vector<double, 11 > FixedwingDynamics :: calcuate_dx_dt(matrix::Vector<double, 11 > state , matrix::Vector<double,4 > control_input )

 {  

    g_x.setZero();

    /* 상태 값을 받아오기 */
    const double n = state(0);
    const double e = state(1);
    const double d = state(2);

    const double roll = state(3);
    const double pitch = state(4);
    const double yaw = state(5);
    const double speed = state(6);

    const double p = state(7);
    const double q = state(8);
    const double r = state(9);
    const double at = state(10);

    /*시상수 관련 파라미터 받아오기*/
    const double t_p =m_spec.timeconstant_p;
    const double t_q =m_spec.timeconstant_q;
    const double t_r =m_spec.timeconstant_r;
    const double t_a =m_spec.timeconstant_at;


    /*중력 및 항력 계수 받아오기*/

    const double g = m_spec.g;
    const double k_acc = m_spec.k_acc;


     /*각도 변환 함수 시작 */
    const double c_r = std::cos(roll);
    const double s_r = std::sin(roll);
    const double c_p = std::cos(pitch);
    const double s_p = std::sin(pitch);
    const double c_y = std::cos(yaw);
    const double s_y = std::sin(yaw);
    const double tan_p = std::tan(pitch);


     /* dx/dt=f(x)+g(x)u 에서 f(x)관련 내용 연산 */

    f_x(0) = speed * c_p* c_y ;
    f_x(1) = speed * c_p * s_y;
    f_x(2) = -speed *s_p;

    f_x(3) = p + (q*s_r*tan_p)+(r*c_r*tan_p);
    f_x(4) = (q*c_r)-(r*s_r);
    f_x(5) = ((q*s_r)+(r*c_r))/c_p ; 
    f_x(6) = -g * s_p + at - (k_acc* (speed * speed ));

    f_x(7) = (-1 / t_p) * p;
    f_x(8) = (-1 / t_q) * q;
    f_x(9) = (-1 / t_r) * r;
    f_x(10) =(-1 / t_a) * at; 


    g_x(7, 0) = 1 / t_p  ;
    g_x(8, 1) = 1 / t_q  ;
    g_x(9,2)  = 1 / t_r  ;
    g_x(10,3) = 1 / t_a  ;


    dx_dt=f_x + (g_x*control_input);

    return dx_dt ; 





 }
         