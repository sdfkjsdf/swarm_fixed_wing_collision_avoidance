#include <matrix/math.hpp>
#include <matrix/integration.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h> 
#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>
#include <propagation/StatePropagation.h>
#include <Fixed_wing_aircraft_model/CurrentState.h>
#include <controller/GainParameter.h>
#include <controller/NdiParameter.h>
#include <controller/MainController.h>
#include <cstdio>
#include <iostream> 


int main() {
    /*각도 출력을 위한 로직*/
    constexpr double kPi = 3.14159265358979323846;
    auto rad2deg = [kPi](double r) { return r * 180.0 / kPi; };

    // 벡터를 생성 후 인덱싱으로 값을 집어넣는 법 //

        // matrix::Vector<자료형, 차원 수 > 변수 이름  = matrix::zeros<float, 행의 갯수, 열의 갯수 >();
        
        // example
            // matrix::Vector<float, 5> x = matrix::zeros<float, 5, 1>();
            // -> 모든 원소가 0인 5차원 열벡터 생성
        
            // 만약에 지금 index으로 특정 값을 넣고 싶다고 하면은
            // x(0) = 1.0f;  // 1번째 원소에 1.0f 대입
            // x(1) = 2.0f;  // 2번째 원
            // x(2) = 3.0f;  // 3번째 원소에 3.0f 대입 이런 식으로 가능하다


   
    // 행렬을 생성 후 값으로 집어 넣는 법 //

        // matrix::Matrix<자료형,행의 갯수, 열의 갯수 > A = matrix::zeros<자료형, 행의 갯수, 열의 갯수 >(); //

        // A(0,0) = 1.f;  A(0,1) = 2.f;  A(0,2) = 3.f;
        // A(1,0) = 4.f;  A(1,1) = 5.f;  A(1,2) = 6.f;
        // A(2,0) = 7.f;  A(2,1) = 8.f;  A(2,2) = 9.f;

        //인덱스는 0부터

        //A(row, col) 형식


    // 단위행렬을 선언하는 경우 //
    
        // matrix::SquareMatrix<float, N> I = matrix::eye<float, N>();

       
    // 벡터에 벡터를  일괄 대입 (Matlab: x(1:5)=v) //

        // matrix::Vector<float, 10> x = matrix::zeros<float, 10, 1>();
        // matrix::Vector<float, 5>  v = matrix::zeros<float, 5, 1>();
        
        // x.slice<a, b>(c, 0) = v 에서 각 기호 의미
            // a : 가져올(또는 덮어쓸) 블록의 행 개수 (rows)
            // b : 가져올(또는 덮어쓸) 블록의 열 개수 (cols)
            // c : x에서 붙여넣기가 시작될 행 인덱스 (start row)
            

    // 벡터를 행렬에 일괄 대입 

        // Matlab: A(a:b, c) (행 구간 + 특정 열 1개)
            // A.slice< (b-a+1), 1 >(a-1, c-1) = v;

        // Matlab: A(a, b:c) (특정 행 1개 + 열 구간)
            // A.slice< 1, (c-b+1) >(a-1, b-1) = v;
        
        // Matlab: A(a:b, c:d) (행 구간 + 열 구간)
            // A.slice< (b-a+1), (d-c+1) >(a-1, c-1) = v;


    //-------------------------------------------------------------------------------------//

    
    FixedwingDynamics my_model( g_FixedwingSpec);
    MainController controller(g_FixedwingSpec, g_GainParameter, g_NdiParameter);
    

    CurrentState state;
    state.set_speed(g_FixedwingSpec.min_speed);      // get/set 함수 사용!
    state.set_at(g_FixedwingSpec.at_min/2);

     CurrentState state_setpoint;
     state_setpoint.set_speed( (g_FixedwingSpec.min_speed+g_FixedwingSpec.max_speed )/2);
     state_setpoint.set_north_position(1000);
     state_setpoint.set_east_position(200.0);
     state_setpoint.set_down(-5000.0);

    matrix::Vector<double, 11> target_setpoint = state_setpoint.get_total_state();


    
    

    double t = 0.0;      // 현재 시간
    double dt = 0.01;    // 시간 간격 (0.01초)

    ProPagation propagator; /* 적분기 생성 */

   


/* test 문장 */
    while(t<100 )
    {
    // 3. 결과 업데이트
    matrix::Vector<double, 11> x = state.get_total_state();
    matrix::Vector<double, 4> control_input{} ;
    control_input = controller.calculate_control_input(
            x ,
            target_setpoint
        );
    matrix::Vector<double, 11> x_next = propagator.step(my_model, x, control_input, t, dt);
    state.set_total_state(x_next);
    t += dt;    // 시간 업데이트

    std::cout << "t=" << t
          << " north=" << x_next(0)
          << " easr="  << x_next(1)
          << " down="  << x_next(2)
          << " roll="  << rad2deg(x_next(3))
          << " pitch=" << rad2deg(x_next(4))
          << " speed=" << x_next(6)
          << std::endl;

    }

   





    









}