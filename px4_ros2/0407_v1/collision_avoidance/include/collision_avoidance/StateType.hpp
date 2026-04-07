#pragma once
#include <array>
#include <cstddef>   
#include <cstdint>
#include <collision_avoidance/spsc_queue.hpp>
#include <Eigen/Dense>




static constexpr size_t kMaxAgents = 8;

namespace StateType
{


    struct State_for_Control
        {      
            std::array<float, 3> position = {0.0f, 0.0f, 0.0f};
            std::array<float, 3> velocity = {0.0f, 0.0f, 0.0f}; 
            std::array<float, 3> position_variance = {0.0f, 0.0f, 0.0f}; 
            std::array<float, 3> velocity_variance = {0.0f, 0.0f, 0.0f}; 
            double timestamp = 0; 
            int check_vehicle_id =0;
            bool update_state=false;
        };



    struct Total_state_for_Control {
        std::array<State_for_Control, kMaxAgents> agents{};
        double timestamp = 0.0;
        int num_agents = 0;

    };


    /*멀트스레드에 주기 위한 자료형 정의*/
    using InputQueue = SpscQueue <Total_state_for_Control, kMaxAgents>; 
    using Check_update = std::array <bool,kMaxAgents >;



    /*rt 내부에서 지금 쓰는 행렬 미리 사전에 정의*/
    using Total_State_Matrix = Eigen::Matrix<float,6,kMaxAgents>;
    using State_Vector=Eigen::Matrix<float,6,1>;
















}/*namespace영역 종료*/
    



