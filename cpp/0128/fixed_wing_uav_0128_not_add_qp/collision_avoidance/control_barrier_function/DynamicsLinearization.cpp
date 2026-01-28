#include <collision_avoidance/control_barrier_function/DynamicsLinearization.h>


DynamicsLinearization::DynamicsLinearization(const FixedwingSpec_t& spec,const double dt )
:m_spec(spec),
m_dt(dt)
{};


void DynamicsLinearization::calculate_fx_gx_Jf_x(const matrix::Vector<double,11>& state)

    {

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

        m_f_x(0) = speed * c_p* c_y ;
        m_f_x(1) = speed * c_p * s_y;
        m_f_x(2) = -speed *s_p;

        m_f_x(3) = p + (q*s_r*tan_p)+(r*c_r*tan_p);
        m_f_x(4) = (q*c_r)-(r*s_r);
        m_f_x(5) = ((q*s_r)+(r*c_r))/c_p ; 
        m_f_x(6) = -g * s_p + at - (k_acc* (speed * speed ));

        m_f_x(7) = (-1 / t_p) * p;
        m_f_x(8) = (-1 / t_q) * q;
        m_f_x(9) = (-1 / t_r) * r;
        m_f_x(10) =(-1 / t_a) * at; 


        m_g_x(7, 0) = 1 / t_p  ;
        m_g_x(8, 1) = 1 / t_q  ;
        m_g_x(9,2)  = 1 / t_r  ;
        m_g_x(10,3) = 1 / t_a  ;
        
        /*f(x)에 대한 자코비안 연산 */

        /* f(0) = V * cos(θ) * cos(ψ) */
        m_Jf_x(0, 4) = -speed * s_p * c_y;      /* ∂f(0)/∂θ */
        m_Jf_x(0, 5) = -speed * c_p * s_y;      /* ∂f(0)/∂ψ */
        m_Jf_x(0, 6) = c_p * c_y;               /* ∂f(0)/∂V */
        
        /* f(1) = V * cos(θ) * sin(ψ) */
        m_Jf_x(1, 4) = -speed * s_p * s_y;      /* ∂f(1)/∂θ */
        m_Jf_x(1, 5) = speed * c_p * c_y;       /* ∂f(1)/∂ψ */
        m_Jf_x(1, 6) = c_p * s_y;               /* ∂f(1)/∂V */
        
        /* f(2) = -V * sin(θ) */
        m_Jf_x(2, 4) = -speed * c_p;            /* ∂f(2)/∂θ */
        m_Jf_x(2, 6) = -s_p;                    /* ∂f(2)/∂V */
        
        /* f(3) = p + q*sin(φ)*tan(θ) + r*cos(φ)*tan(θ) */
        m_Jf_x(3, 3) = q * c_r * tan_p - r * s_r * tan_p;       /* ∂f(3)/∂φ */
        m_Jf_x(3, 4) = (q * s_r + r * c_r) / (c_p * c_p);       /* ∂f(3)/∂θ */
        m_Jf_x(3, 7) = 1.0;                                      /* ∂f(3)/∂p */
        m_Jf_x(3, 8) = s_r * tan_p;                              /* ∂f(3)/∂q */
        m_Jf_x(3, 9) = c_r * tan_p;                              /* ∂f(3)/∂r */
        
        /* f(4) = q*cos(φ) - r*sin(φ) */
        m_Jf_x(4, 3) = -q * s_r - r * c_r;      /* ∂f(4)/∂φ */
        m_Jf_x(4, 8) = c_r;                     /* ∂f(4)/∂q */
        m_Jf_x(4, 9) = -s_r;                    /* ∂f(4)/∂r */
        
        /* f(5) = (q*sin(φ) + r*cos(φ)) / cos(θ) */
        m_Jf_x(5, 3) = (q * c_r - r * s_r) / c_p;               /* ∂f(5)/∂φ */
        m_Jf_x(5, 4) = (q * s_r + r * c_r) * tan_p / c_p;       /* ∂f(5)/∂θ */
        m_Jf_x(5, 8) = s_r / c_p;                                /* ∂f(5)/∂q */
        m_Jf_x(5, 9) = c_r / c_p;                                /* ∂f(5)/∂r */
        
        /* f(6) = -g*sin(θ) + at - k_acc*V² */
        m_Jf_x(6, 4) = -g * c_p;                /* ∂f(6)/∂θ */
        m_Jf_x(6, 6) = -2.0 * k_acc * speed;    /* ∂f(6)/∂V */
        m_Jf_x(6, 10) = 1.0;                    /* ∂f(6)/∂at */
        
        /* f(7) = (-1/τp) * p */
        m_Jf_x(7, 7) = -1.0 / t_p;
        
        /* f(8) = (-1/τq) * q */
        m_Jf_x(8, 8) = -1.0 / t_q;
        
        /* f(9) = (-1/τr) * r */
        m_Jf_x(9, 9) = -1.0 / t_r;
        
        /* f(10) = (-1/τa) * at */
        m_Jf_x(10, 10) = -1.0 / t_a;



    }

void DynamicsLinearization::calculate_C(const matrix::Vector<double, 11 >& state,const matrix::Matrix<double,11,11>& m_Jf_x)
    {

    m_C=matrix::Vector<double, 11 >(  m_f_x -   matrix::Vector<double, 11 >(m_Jf_x*state ) );

    }
void DynamicsLinearization::calculate_Ad_Bd()
{
    const double T = m_dt;
    const double T2 = T * T;
    const double T3 = T2 * T;
    
    matrix::Matrix<double, 11, 11> I;
    I.setIdentity();
    
    matrix::Matrix<double, 11, 11> A = m_Jf_x;
    matrix::Matrix<double, 11, 11> A2 = A * A;
    matrix::Matrix<double, 11, 11> A3 = A2 * A;
    
    /* 3차 테일러 근사 */
    m_Ad = I + A * T + A2 * (T2 / 2.0) + A3 * (T3 / 6.0);
    
    /* Bd 적분 근사: ∫₀ᵀ e^(Aτ) dτ ≈ IT + AT²/2 + A²T³/6 */
    m_Bd = I * T + A * (T2 / 2.0) + A2 * (T3 / 6.0);
}

/* 정 안되면 해야 할 내용 현재 4차까지 확장한 내용임*/
/*
void DynamicsLinearization::calculate_Ad_Bd()
{
    const double T = m_dt;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T; // T4 추가

    matrix::Matrix<double, 11, 11> I;
    I.setIdentity();

    matrix::Matrix<double, 11, 11> A = m_Jf_x;
    matrix::Matrix<double, 11, 11> A2 = A * A;
    matrix::Matrix<double, 11, 11> A3 = A2 * A;
    matrix::Matrix<double, 11, 11> A4 = A3 * A; // A4 추가

    
  
    matrix::Matrix<double, 11, 11> Psi = I * T 
                                       + A * (T2 / 2.0) 
                                       + A2 * (T3 / 6.0) 
                                       + A3 * (T4 / 24.0); // 핵심: 이 항이 있어야 선회 가능

    m_Bd = Psi * m_g_x; 
}
*/