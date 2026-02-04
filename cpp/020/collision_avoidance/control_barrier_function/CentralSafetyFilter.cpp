#include <collision_avoidance/control_barrier_function/CentralSafetyFilter.h>

CentralSafetyFilter::CentralSafetyFilter(const FixedwingSpec_t& g_FixedwingSpec, const double dt)
: m_spec(g_FixedwingSpec),
  m_dt(dt),
  m_DynamicsLinearization(g_FixedwingSpec, dt),
  m_CentralSafetyBarrierCertificate()
{
}

Qpinequality CentralSafetyFilter::calculate_inequality(
    const matrix::Vector<double,11>& state_i,
    const matrix::Vector<double,11>& state_j,
    const int i,
    const int j)
{  
    /*초기화 및 바로 할당*/
    Qpinequality out{};
    out.m_i = i;
    out.m_j = j;

    /* ZOH 선형화 */
    m_DynamicsLinearization.calculate_zoh_matrix(state_i);
        const matrix::Matrix<double,11,11> temp_Ad_i = m_DynamicsLinearization.get_Ad();
        const matrix::Matrix<double,11,4>  temp_BdB_i = m_DynamicsLinearization.get_BdB();
        const matrix::Vector<double,11>    temp_BdC_i= m_DynamicsLinearization.get_BdC();

    m_DynamicsLinearization.calculate_zoh_matrix(state_j);
        const matrix::Matrix<double,11,11> temp_Ad_j = m_DynamicsLinearization.get_Ad();
        const matrix::Matrix<double,11,4>  temp_BdB_j = m_DynamicsLinearization.get_BdB();
        const matrix::Vector<double,11>    temp_BdC_j = m_DynamicsLinearization.get_BdC();


    /*CBF 연산*/
    m_CentralSafetyBarrierCertificate.calculate_h_x_i_gradient_h_x_i(state_i,state_j);

        const double temp_h_ij = m_CentralSafetyBarrierCertificate.get_h_x();
        const matrix::Matrix<double,1,11> temp_gradient_h_x_i= m_CentralSafetyBarrierCertificate.get_gradient_h_x_i();
        const matrix::Matrix<double,1,11> temp_gradient_h_x_j= m_CentralSafetyBarrierCertificate.get_gradient_h_x_j();

        const matrix::Matrix<double,1,4> temp_lgh_i=temp_gradient_h_x_i*temp_BdB_i;
        const matrix::Matrix<double,1,4> temp_lgh_j=temp_gradient_h_x_j*temp_BdB_j;

    /*제약조건 인수 할당*/
    out.A_i_vector(0,0)=temp_lgh_i(0,1);
    out.A_i_vector(0,1)=temp_lgh_i(0,2);
    out.A_i_vector(0,2)=temp_lgh_i(0,3);

    out.A_j_vector(0,0)=temp_lgh_j(0,1);
    out.A_j_vector(0,1)=temp_lgh_j(0,2);
    out.A_j_vector(0,2)=temp_lgh_j(0,3);

    out.B_ij = -(r * temp_h_ij)
               -(temp_gradient_h_x_i * (temp_Ad_i * state_i +temp_BdC_i -state_i))(0, 0)
               -(temp_gradient_h_x_j * (temp_Ad_j * state_j +temp_BdC_j -state_j))(0, 0) ;



    // ...나머지 구현...
    return out;
}

void CentralSafetyFilter::calculate_total_qp_matrix(const matrix::Vector<double,11>& state_0,
                                                    const matrix::Vector<double,11>& state_1,
                                                    const matrix::Vector<double,11>& state_2,
                                                    const matrix::Vector<double,11>& state_3,
                                                    const matrix::Vector<double,11>& state_4)
    {
        /*일단 편의를 위해서 이런 식으로 묶기 나중에 지금 수정이 가능함*/
        std::vector<matrix::Vector<double, 11>> total_states(5); /*여기서 5는 총 에이전트의 수*/

        total_states[0]=state_0;
        total_states[1]=state_1;
        total_states[2]=state_2;
        total_states[3]=state_3;
        total_states[4]=state_4;


       int row = 0;
        for(int i=0;i<N;i++){
        for(int j=i+1;j<N;j++){

            Qpinequality ineq = calculate_inequality(total_states[i], total_states[j], i, j);

            const int ci = U_DIM * i;
            const int cj = U_DIM * j;

            /*i 블록 채우기*/
            for(int k=0;k<U_DIM;k++){
            m_central_A_matrix(row, ci + k) = ineq.A_i_vector(0, k);
            }
            /*j 블록 채우기*/ 
            for(int k=0;k<U_DIM;k++){
            m_central_A_matrix(row, cj + k) = ineq.A_j_vector(0, k);
            }

            m_central_B_vector(row, 0) = ineq.B_ij;

            row++;
        }
}
        

        











    }