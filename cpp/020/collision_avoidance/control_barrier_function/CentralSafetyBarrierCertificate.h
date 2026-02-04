#ifndef CENTRALSAFETYBARRIERCERTIFICATE_H
#define CENTRALSAFETYBARRIERCERTIFICATE_H
#include<matrix/math.hpp>



/*

h_ij(P_ij(k+1).V_ij(k+1))=h_ij(P_ij(k),V_ij(k))
                          + ∂h_ij/∂x_i* (x_i(k+1)=x_i(k)) 
                          + ∂h_ij/∂x_j* (x_j(k+1)-x_j(k))



*/



class CentralSafetyBarrierCertificate

{
  public:
  CentralSafetyBarrierCertificate()=default;
  ~CentralSafetyBarrierCertificate()=default;


  /* 지금 일단 수정의 용의성을 위해서 전체 state으로 받는 것임 나중에 ts 메시지로도 수정이 가능하기는 함*/

  void calculate_h_x_i_gradient_h_x_i( const matrix::Vector<double,11>& state_i,const matrix::Vector<double,11>& state_j);



  matrix::Matrix<double,3,11> calculate_grad_pij_x();

  matrix::Matrix<double,3,11> calculate_grad_vij_x(const double speed,
                            const double c_p,
                            const double s_p,
                            const double c_y,
                            const double s_y);
  

  double get_h_x(){return m_h_x;};
  matrix::Vector<double,3> get_gradient_p_h(){return m_gradient_p_ij_h; };
  matrix::Vector<double,3> get_gradient_v_h(){return m_gradient_v_ij_h; };

  matrix::Matrix<double,1,11> get_gradient_h_x_i(){return m_gradient_h_x_i; };
  matrix::Matrix<double,1,11> get_gradient_h_x_j(){return m_gradient_h_x_j; };

  
         
  private:
    /*cbf 정의 관련 내용*/
    const double m_r_p=10.0;
    const double m_critical_distance=30.0;


    /* h(x)에 대한 편미분 관련 내용*/
    double m_h_x=0.0;
    matrix::Vector<double,3>m_gradient_p_ij_h{};
    matrix::Vector<double,3>m_gradient_v_ij_h{};

    /* 고정익 모델에 대한 편미분 관련 내용*/
    matrix::Matrix<double,3,11> m_grad_pij_x_i{};
    matrix::Matrix<double,3,11> m_grad_vij_x_i{};

    matrix::Matrix<double,3,11> m_grad_pij_x_j{};
    matrix::Matrix<double,3,11> m_grad_vij_x_j{};


    matrix::Matrix<double,1,11>m_gradient_h_x_i{};
    matrix::Matrix<double,1,11>m_gradient_h_x_j{}; 


};


#endif