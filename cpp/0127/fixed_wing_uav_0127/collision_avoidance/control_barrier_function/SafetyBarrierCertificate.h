#ifndef SAFETYBARRIERCERTIFICATE_H
#define SAFETYBARRIERCERTIFICATE_H
#include<matrix/math.hpp>



class SafetyBarrierCertificate

{
  public:
  SafetyBarrierCertificate(const double& r_p, const double& critical_distance);
  ~SafetyBarrierCertificate()=default;


  /* 지금 일단 수정의 용의성을 위해서 전체 state으로 받는 것임 나중에 ts 메시지로도 수정이 가능하기는 함*/

  void calculate_h_grad_h( const matrix::Vector<double,11>& state_i,const matrix::Vector<double,11>& state_j);

  double get_h_x(){return m_h_x;};
  matrix::Vector<double,3> get_gradient_p_h(){return m_gradient_p_h; };
   matrix::Vector<double,3> get_gradient_v_h(){return m_gradient_v_h; };



 
  private:
  double m_r_p=0.0;
  double m_critical_distance=0.0;
  double m_h_x=0.0;
  matrix::Vector<double,3>m_gradient_p_h{};
  matrix::Vector<double,3>m_gradient_v_h{};
  matrix::Vector<double,11>m_gradient_h_x{}; 


};


#endif