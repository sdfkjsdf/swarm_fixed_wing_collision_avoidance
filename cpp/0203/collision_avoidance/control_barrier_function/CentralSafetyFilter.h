#ifndef CENTRALSAFETYFILTER_H
#define CENTRALSAFETYFILTER_H
#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>
#include <collision_avoidance/control_barrier_function/CentralSafetyBarrierCertificate.h>
#include <propagation/DynamicsLinearization.h>
#include <matrix/math.hpp>
#include <vector>





struct Qpinequality
{  
    matrix::Matrix<double,1,3> A_i_vector{};
    matrix::Matrix<double,1,3> A_j_vector{};
    double B_ij=0.0;
    int m_i=0;
    int m_j=0;

};

/* 여기서 지금 [A_i;A_j]x>=Bij*/



class CentralSafetyFilter

{

    public:
    CentralSafetyFilter( const FixedwingSpec_t& g_FixedwingSpec, const double dt);

    ~CentralSafetyFilter()=default;
    
    /*해당 기능은 지금 i,j가 정해졌을 때의 값을 구하는 것임*/
    Qpinequality calculate_inequality(const matrix::Vector<double,11>& state_i,const matrix::Vector<double,11>& state_j,const int i,const int j);

    void calculate_total_qp_matrix(const matrix::Vector<double,11>& state_0,
                                   const matrix::Vector<double,11>& state_1,
                                   const matrix::Vector<double,11>& state_2,
                                   const matrix::Vector<double,11>& state_3,
                                   const matrix::Vector<double,11>& state_4);

    
     matrix::Matrix<double,10,15> get_central_A_matrix(){return m_central_A_matrix;}
     matrix::Matrix<double,10,1>  get_central_B_vector(){return m_central_B_vector;}



    private:
    

    /*매개변수 생성자 동시 저장*/
     FixedwingSpec_t m_spec;
     double m_dt;

    /*cbf 용 class-k 함수 */
    const int N=5;
    const int U_DIM = 3;
    const double r=10.0;

    /*모듈을 내장시키기*/
     
     DynamicsLinearization m_DynamicsLinearization; 
     CentralSafetyBarrierCertificate  m_CentralSafetyBarrierCertificate;




    /*최종적으로 출력하는 것을 선언*/
    matrix::Matrix<double,10,15> m_central_A_matrix{};
    matrix::Matrix<double,10,1>  m_central_B_vector{};

    


};








#endif