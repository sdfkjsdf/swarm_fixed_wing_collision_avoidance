#ifndef SAFETYBARRIERCERTIFICATE_H
#define SAFETYBARRIERCERTIFICATE_H
#include<matrix/math.hpp>


/*

h_ij(P_ij(k+1).V_ij(k+1))=h_ij(P_ij(k),V_ij(k))
                          + ∂h_ij/∂x_i* (x_i(k+1)=x_i(k)) 
                          + ∂h_ij/∂x_j* (x_j(k+1)-x_j(k))



*/






class SafetyBarrierCertificate

{
  public:
  SafetyBarrierCertificate(const double& r_p, const double& critical_distance);
  ~SafetyBarrierCertificate()=default;


  /* 지금 일단 수정의 용의성을 위해서 전체 state으로 받는 것임 나중에 ts 메시지로도 수정이 가능하기는 함*/

  void calculate_h_grad_h( const matrix::Vector<double,11>& state_i,const matrix::Vector<double,11>& state_j);

  double get_h_x(){return m_h_x;};
  matrix::Vector<double,3> get_gradient_p_h(){return m_gradient_p_ij_h; };
  matrix::Vector<double,3> get_gradient_v_h(){return m_gradient_v_ij_h; };

  matrix::Matrix<double,3,11> calculate_grad_vij_x(const double speed,
                                                   const double c_p,
                                                   const double s_p,
                                                   const double c_y,
                                                   const double s_y)
          {  
            matrix::Matrix<double,3,11> grad_vij_x{};

             grad_vij_x(0,4)=-speed*s_p*c_y;
             grad_vij_x(1,4)=-speed*s_p*s_y;
             grad_vij_x(2,4)=-speed*c_p;

             grad_vij_x(0,5)=-speed*c_p*s_y;
             grad_vij_x(1,5)= speed*c_p*c_y;

             grad_vij_x(0,6)= c_p*c_y;
             grad_vij_x(1,6)= c_p*s_y;
             grad_vij_x(2,6)= -s_p;

             return grad_vij_x;


          };



 
  private:
  double m_r_p=0.0;
  double m_critical_distance=0.0;
  double m_h_x=0.0;
  matrix::Vector<double,3>m_gradient_p_ij_h{};
  matrix::Vector<double,3>m_gradient_v_ij_h{};
  matrix::Matrix<double,1,11>m_gradient_h_x_i{};
  matrix::Matrix<double,1,11>m_gradient_h_x_j{}; 


};


#endif