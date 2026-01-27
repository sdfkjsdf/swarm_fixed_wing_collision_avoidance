#include <collision_avoidance/control_barrier_function/SafetyBarrierCertificate.h>
#include<cmath>

SafetyBarrierCertificate::SafetyBarrierCertificate(const double& r_p, const double& critical_distance)
:m_r_p(r_p),
m_critical_distance(critical_distance){}



void SafetyBarrierCertificate:: calculate_h_grad_h(const matrix::Vector<double,11>& state_i,const matrix::Vector<double,11>& state_j)
{

matrix::Vector<double,3> position_i{};
matrix::Vector<double,3> position_j{};

matrix::Vector<double,3> velocity_i{};
matrix::Vector<double,3> velocity_j{};


/*  상태 변수 받아오기 밎 추출하기*/



const double pitch_i = state_i(4);
const double yaw_i = state_i(5);
const double speed_i = state_i(6);

const double pitch_j = state_j(4);
const double yaw_j = state_j(5);
const double speed_j = state_j(6);


position_i(0)=state_i(0);
position_i(1)=state_i(1);
position_i(2)=state_i(2);
velocity_i(0) =  speed_i * std::cos(pitch_i)* std::cos(yaw_i) ;
velocity_i(1) =  speed_i * std::cos(pitch_i)* std::sin(yaw_i) ;
velocity_i(2) = -speed_i *std::sin(pitch_i) ;

position_j(0)=state_j(0);
position_j(1)=state_j(1);
position_j(2)=state_j(2);

velocity_j(0) =  speed_j * std::cos(pitch_j)* std::cos(yaw_j) ;
velocity_j(1) =  speed_j * std::cos(pitch_j)* std::sin(yaw_j) ;
velocity_j(2) = -speed_j *std::sin(pitch_j) ;




matrix::Vector<double,3> p_ij{};
matrix::Vector<double,3> v_ij{};

p_ij=position_i-position_j;
v_ij=velocity_i-velocity_j;

double norm_p_ij=p_ij.norm();
double norm_pij_sp=p_ij.norm_squared();
double dot_pv_ij=p_ij.dot(v_ij);


/* 지금 해당 부분은  

m_h_x=(m_r_p/norm_p_ij)*dot_pv_ij+(norm_p_ij-m_critical_distance);
m_gradient_p_h=(m_r_p/norm_p_ij)*(v_ij-(dot_pv_ij/norm_pij_sp)*p_ij )+(p_ij/norm_p_ij);
m_gradient_v_h=m_r_p*p_ij/norm_p_ij;


m_gradient_h_x.slice<3,1>(0,0)=m_gradient_p_h;
m_gradient_h_x.slice<3,1>(3,0)=m_gradient_v_h;


*/

}