#include <matrix/math.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h> 
#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>
#include <propagation/StatePropagation.h>
#include <controller/GainParameter.h>
#include <controller/NdiParameter.h>
#include <iostream> 
#include <make_csv/FixedWingLog.h>
#include <vector>
#include <fstream>
#include<sstream>
#include <random>
#include <formation_guidance/FlockingParameter.h>
#include <formation_guidance/LeaderFollowerParameter.h>
#include <formation_guidance/WingmanPositoin.h>
#include <agent/Agent.h>

int main() {
    /*========================================
     *  상수 및 설정
     *========================================*/
    constexpr double kPi = 3.14159265358979323846;
    const int total_vehicle_num = 5;
    const double dt = 0.01;
    const double t_end = 400.0;

    /*========================================
     *  Agent 생성
     *========================================*/
    std::vector<Agent> agents;
    for (int i = 0; i < total_vehicle_num; i++) {
        agents.emplace_back(i, g_FixedwingSpec, g_Flockingparameter, 
                           g_LeaderFollowerParameter, g_GainParameter, g_NdiParameter, g_APFParameter);
        agents[i].set_collision_avoidance_mode(false);
    }

   

    /*========================================
     *  Simulator 컴포넌트 (Agent 외부)
     *========================================*/
    std::vector<FixedwingDynamics> dynamics;
    for (int i = 0; i < total_vehicle_num; i++) {
        dynamics.emplace_back(g_FixedwingSpec);
    }
    ProPagation propagator;

    
std::vector<matrix::Vector<double, 11>> states(total_vehicle_num);

std::ifstream init_file("../../build_initial_condition/initial_conditions.txt");
if (!init_file.is_open()) {
    std::cerr << "Failed to open initial_conditions.txt!" << std::endl;
    return 1;
}

std::string line;
int loaded_count = 0;

while (std::getline(init_file, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#') continue;
    
    std::stringstream ss(line);
    int id;
    double n, e, d, yaw, speed;
    char comma;
    
    ss >> id >> comma >> n >> comma >> e >> comma >> d >> comma >> yaw >> comma >> speed;
    
    if (id >= 0 && id < total_vehicle_num) {
        states[id].setZero();
        states[id](0) = n;      // North
        states[id](1) = e;      // East
        states[id](2) = d;      // Down
        states[id](5) = yaw;    // Yaw
        states[id](6) = speed;  // Speed
        states[id](10) = (g_FixedwingSpec.at_min + g_FixedwingSpec.at_max) / 2.0;  // Throttle
        
        agents[id].receive_state_from_sim(states[id]);
        
        loaded_count++;
        std::cout << "Agent " << id << " loaded: "
                  << "N=" << n << " E=" << e 
                  << " D=" << d << " Yaw=" << yaw << std::endl;
    }
}
init_file.close();

std::cout << "Loaded " << loaded_count << " agents from file." << std::endl;
    /*========================================
     *  Setpoint 설정
     *========================================*/
    matrix::Vector<double, 11> target_setpoint;
    target_setpoint.setZero();
    
    for (int i = 0; i < total_vehicle_num; i++) {
        agents[i].receive_state_setpoint(target_setpoint);
    }

    /*========================================
     *  Wingman 위치 설정
     *========================================*/
    WingManPostion build_wingmanposition;
    const double wingman_distance = 100.0;
    matrix::Vector<double, 2> wingman_parallel;
    wingman_parallel(0) = 100.0;
    wingman_parallel(1) = 0.0;

    const double right_angle = 45.0 * kPi / 180;
    const double left_angle = -45.0 * kPi / 180;

    std::vector<matrix::Vector<double, 2>> wingman_positions(total_vehicle_num);
    wingman_positions[0].setZero();  // Leader
    wingman_positions[1] = build_wingmanposition.calculate_wingman_setposition(wingman_distance, wingman_parallel, right_angle, 1.0);
    wingman_positions[2] = build_wingmanposition.calculate_wingman_setposition(wingman_distance, wingman_parallel, right_angle, 2.0);
    wingman_positions[3] = build_wingmanposition.calculate_wingman_setposition(wingman_distance, wingman_parallel, left_angle, 1.0);
    wingman_positions[4] = build_wingmanposition.calculate_wingman_setposition(wingman_distance, wingman_parallel, left_angle, 2.0);

    for (int i = 0; i < total_vehicle_num; i++) {
        agents[i].receive_wingman_setpoint(wingman_positions[i]);
    }

    /*========================================
     *  지상국 메시지 생성
     *========================================*/
    std::vector<TcMessage_t> tc_messages(total_vehicle_num);

    /*========================================
     *  CSV 로그 생성
     *========================================*/
    std::vector<FixedWingLog> histories;
    for (int i = 0; i < total_vehicle_num; i++) {
        histories.emplace_back(i);
    }

    /*========================================
     *  시뮬레이션 루프
     *========================================*/
    double t = 0.0;

    while (t < t_end)
    {
        // 1. CSV 데이터 저장
        for (int i = 0; i < total_vehicle_num; i++) {
            histories[i].addData(t, states[i]);
        }

        // 2. 지상국 → Agent 통신 (TC 메시지)
        tc_messages[0].broadcast_tc_message(0, t, true, true);   // Leader
        for (int i = 1; i < total_vehicle_num; i++) {
            tc_messages[i].broadcast_tc_message(i, t, true, false);  // Followers
        }
        for (int i = 0; i < total_vehicle_num; i++) {
            agents[i].receive_tc_message(tc_messages[i]);
        }

        // 3. Agent TS 메시지 생성
        for (int i = 0; i < total_vehicle_num; i++) {
            agents[i].generate_ts_message(t);
        }

        // 4. Agent 간 통신 수신 (자신 제외한 4개 메시지)
        agents[0].receive_ts_message(
            agents[1].broadcast_ts_message(), agents[2].broadcast_ts_message(),
            agents[3].broadcast_ts_message(), agents[4].broadcast_ts_message());
        agents[1].receive_ts_message(
            agents[0].broadcast_ts_message(), agents[2].broadcast_ts_message(),
            agents[3].broadcast_ts_message(), agents[4].broadcast_ts_message());
        agents[2].receive_ts_message(
            agents[0].broadcast_ts_message(), agents[1].broadcast_ts_message(),
            agents[3].broadcast_ts_message(), agents[4].broadcast_ts_message());
        agents[3].receive_ts_message(
            agents[0].broadcast_ts_message(), agents[1].broadcast_ts_message(),
            agents[2].broadcast_ts_message(), agents[4].broadcast_ts_message());
        agents[4].receive_ts_message(
            agents[0].broadcast_ts_message(), agents[1].broadcast_ts_message(),
            agents[2].broadcast_ts_message(), agents[3].broadcast_ts_message());

        // 5. 제어 입력 계산 & 시뮬레이션
        for (int i = 0; i < total_vehicle_num; i++) {
            // Agent가 제어 입력 계산
            matrix::Vector<double, 4> control_input = agents[i].send_control_input_to_sim();
            
            // Simulator가 적분
            states[i] = propagator.step(dynamics[i], states[i], control_input, t, dt);
            
            // 새 상태를 Agent에게 전달
            agents[i].receive_state_from_sim(states[i]);
        }

        // 6. 시간 증가
        t += dt;
    }

    /*========================================
     *  CSV 파일 저장
     *========================================*/
    for (int i = 0; i < total_vehicle_num; i++) {
        std::string filename = "../../make_csv/agent_" + std::to_string(i) + ".csv";
        histories[i].saveToCSV(filename);
        std::cout << "Agent " << i << " CSV saved!" << std::endl;
    }

    /*========================================
     *  스펙 JSON 저장
     *========================================*/
    std::ofstream config("../../make_csv/aircraft_spec.json");
    config << "{\n"
           << "  \"max_speed\": " << g_FixedwingSpec.max_speed << ",\n"
           << "  \"min_speed\": " << g_FixedwingSpec.min_speed << ",\n"
           << "  \"max_roll\": " << g_FixedwingSpec.max_roll * 180.0 / M_PI << ",\n"
           << "  \"min_roll\": " << g_FixedwingSpec.min_roll * 180.0 / M_PI << ",\n"
           << "  \"max_pitch\": " << g_FixedwingSpec.max_pitch * 180.0 / M_PI << ",\n"
           << "  \"min_pitch\": " << g_FixedwingSpec.min_pitch * 180.0 / M_PI << ",\n"
           << "  \"max_p\": " << g_FixedwingSpec.p_max * 180.0 / M_PI << ",\n"
           << "  \"min_p\": " << g_FixedwingSpec.p_min * 180.0 / M_PI << ",\n"
           << "  \"max_q\": " << g_FixedwingSpec.q_max * 180.0 / M_PI << ",\n"
           << "  \"min_q\": " << g_FixedwingSpec.q_min * 180.0 / M_PI << ",\n"
           << "  \"max_r\": " << g_FixedwingSpec.r_max * 180.0 / M_PI << ",\n"
           << "  \"min_r\": " << g_FixedwingSpec.r_min * 180.0 / M_PI << ",\n"
           << "  \"max_at\": " << g_FixedwingSpec.at_max << ",\n"
           << "  \"min_at\": " << g_FixedwingSpec.at_min << "\n"
           << "}\n";
    config.close();

    return 0;
}