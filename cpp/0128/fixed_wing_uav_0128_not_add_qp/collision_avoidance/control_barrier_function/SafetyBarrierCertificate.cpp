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

const double c_p_i = std::cos(pitch_i);
const double s_p_i = std::sin(pitch_i);
const double c_y_i = std::cos(yaw_i);
const double s_y_i = std::sin(yaw_i);

const double pitch_j = state_j(4);
const double yaw_j = state_j(5);
const double speed_j = state_j(6);

const double c_p_j = std::cos(pitch_j);
const double s_p_j = std::sin(pitch_j);
const double c_y_j = std::cos(yaw_j);
const double s_y_j = std::sin(yaw_j);


position_i(0)=state_i(0);
position_i(1)=state_i(1);
position_i(2)=state_i(2);
velocity_i(0) =  speed_i * c_p_i*c_y_i;
velocity_i(1) =  speed_i * c_p_i*s_y_i;
velocity_i(2) = -speed_i * s_p_i;

position_j(0)=state_j(0);
position_j(1)=state_j(1);
position_j(2)=state_j(2);

velocity_j(0) =  speed_j * c_p_j * c_y_j;
velocity_j(1) =  speed_j * c_p_j * s_y_j;
velocity_j(2) = -speed_j * s_p_j;


/* 각자의 정보에 대한 그레디언트 벡터를 구하기*/
/*지금 여기서 일단 개념검증을 위해서 큰 행렬을 쓰는데 나중에 최적화를 하기*/

matrix::Matrix<double,3,11> grad_pij_xi{};
grad_pij_xi(0,0)=1.0;
grad_pij_xi(1,1)=1.0;
grad_pij_xi(2,2)=1.0;

matrix::Matrix<double,3,11> grad_pij_xj =-grad_pij_xi;

matrix::Matrix<double,3,11> grad_vij_xi=calculate_grad_vij_x(speed_i,
                                                             c_p_i,
                                                             s_p_i,
                                                             c_y_i,
                                                             s_y_i);

matrix::Matrix<double,3,11> grad_vij_xj= -calculate_grad_vij_x(speed_j,
                                                             c_p_j,
                                                             s_p_j,
                                                             c_y_j,
                                                             s_y_j);



/*CBF 구현 시작*/


matrix::Vector<double,3> p_ij{};
matrix::Vector<double,3> v_ij{};

p_ij=position_i-position_j;
v_ij=velocity_i-velocity_j;

double norm_p_ij=p_ij.norm();
double norm_pij_sp=p_ij.norm_squared();
double dot_pv_ij=p_ij.dot(v_ij);

m_h_x=(m_r_p/norm_p_ij)*dot_pv_ij+(norm_p_ij-m_critical_distance);
m_gradient_p_ij_h=(m_r_p/norm_p_ij)*(v_ij-(dot_pv_ij/norm_pij_sp)*p_ij )+(p_ij/norm_p_ij);
m_gradient_v_ij_h=m_r_p*p_ij/norm_p_ij;



m_gradient_h_x_i= m_gradient_p_ij_h.transpose() * grad_pij_xi 
                  + m_gradient_v_ij_h.transpose() * grad_vij_xi;


m_gradient_h_x_j= m_gradient_p_ij_h.transpose() * grad_pij_xj 
                  + m_gradient_v_ij_h.transpose() * grad_vij_xj;

}