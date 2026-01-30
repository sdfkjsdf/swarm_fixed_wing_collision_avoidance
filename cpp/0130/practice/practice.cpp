#include <matrix/math.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h> 
#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>
#include <propagation/StatePropagation.h>
#include <controller/GainParameter.h>
#include <controller/NdiParameter.h>
#include <iostream> 
#include <make_csv/FixedWingLog.h>
#include <make_csv/SimulationConfig.h>
#include <vector>
#include <fstream>
#include<sstream>
#include <random>
#include <formation_guidance/FlockingParameter.h>
#include <formation_guidance/LeaderFollowerParameter.h>
#include <formation_guidance/WingmanPositoin.h>
#include <agent/Agent.h>



#include <chrono>

int main() {
   
    constexpr double kPi = 3.14159265358979323846;
    const int total_vehicle_num = 5;
    const double dt = 0.01;
    const double t_end = 300.0;

   /*Agent 생성 */
    std::vector<Agent> agents;
    for (int i = 0; i < total_vehicle_num; i++) {
        agents.emplace_back(i, g_FixedwingSpec, g_Flockingparameter, 
                           g_LeaderFollowerParameter, g_GainParameter, g_NdiParameter, g_APFParameter,dt);
        /*APF 기반 충돌회피 스위치*/
        agents[i].set_collision_avoidance_mode(false);
    }


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


/*초기조건이 담긴 파일을 받아오기 */
while (std::getline(init_file, line)) {
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


    /*단독에 대한 setpoint을 설정*/

    matrix::Vector<double, 11> target_setpoint;
    target_setpoint.setZero();
    target_setpoint(2)=-0.0;
    
    for (int i = 0; i < total_vehicle_num; i++) {
        agents[i].receive_state_setpoint(target_setpoint);
    }


    /*Leader의 벡터 필드 가이던스 생성*/


    

    /*Wingman 위치 설정*/

    WingManPostion build_wingmanposition;
    const double wingman_distance = 50.0;
    matrix::Vector<double, 2> wingman_parallel;
    wingman_parallel(0) = 0.0;
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

    /* 지상국 메시지 설정 */
    std::vector<TcMessage_t> tc_messages(total_vehicle_num);

    /*CSV 로그 생성*/
    std::vector<FixedWingLog> histories;
    for (int i = 0; i < total_vehicle_num; i++) {
        histories.emplace_back(i);
    }

    std::vector<std::ofstream> error_logs;
    for (int i = 0; i < total_vehicle_num; i++) {
        std::string filename = "../../make_csv/zoh_error_agent_" + std::to_string(i) + ".csv";
        error_logs.emplace_back(filename);
        error_logs[i] << "time,error_n,error_e,error_d,error_roll,error_pitch,error_yaw,"
                      << "error_speed,error_p,error_q,error_r,error_at,"
                      << "total_pos_error,total_angle_error\n";
    }







    /*Leader-follower 스위치 및 자료저장*/
    bool leader_follower_switch =false;

    save_simulation_config("../../make_csv/sim_config.json", 
                        leader_follower_switch, 
                        wingman_distance, 
                        right_angle, 
                        left_angle);





    /*시뮬레이션 루프 시작*/

    /*초기 시간 정의 */
    double t = 0.0;

    auto start_time = std::chrono::high_resolution_clock::now();

    while (t < t_end)
    {
        /* CSV에 데이터 저장*/
        for (int i = 0; i < total_vehicle_num; i++) {
            histories[i].addData(t, states[i]);
        }

        /* 2. 지상국 → Agent 통신 (TC 메시지)*/
        tc_messages[0].broadcast_tc_message(0, t, leader_follower_switch, true);   // Leader
        for (int i = 1; i < total_vehicle_num; i++) {
            tc_messages[i].broadcast_tc_message(i, t, leader_follower_switch, false);  // Followers
        }
        for (int i = 0; i < total_vehicle_num; i++) {
            agents[i].receive_tc_message(tc_messages[i]);
        }

        /*3. Agent TS 메시지 생성*/
        for (int i = 0; i < total_vehicle_num; i++) {
            agents[i].generate_ts_message(t);
        }

        /*4. Agent 간 통신 수신 (자신 제외한 4개 메시지)*/
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

        /*5. 제어 입력 계산 & 시뮬레이션*/
        for (int i = 0; i < total_vehicle_num; i++) {
            /*  Agent가 제어 입력 계산*/
            matrix::Vector<double, 4> control_input = agents[i].send_control_input_to_sim();
            /*  ZOH 예측  */
            matrix::Vector<double, 11> state_zoh = agents[i].send_zoh_next_step();
            
            /* Simulator가 RK4 적분*/
            matrix::Vector<double, 11> state_rk4 = propagator.step(dynamics[i], states[i], control_input, t, dt);
            
            /*  오차 계산 및 저장  */
            matrix::Vector<double, 11> error = state_rk4 - state_zoh;
            
           
            
            
            error_logs[i] << std::fixed << std::setprecision(9)
              << t << ","
              << error(0) << "," << error(1) << "," << error(2) << ","
              << error(3) << "," << error(4) << "," << error(5) << ","
              << error(6) << "," << error(7) << "," << error(8) << ","
              << error(9) << "," << error(10) << "\n";
            
            /* 상태 업데이트 (RK4 결과 사용) */
            states[i] = state_rk4;
            
            /*새 상태를 Agent에게 전달*/
            agents[i].receive_state_from_sim(states[i]);
        }

        /* 6. 시간 증가*/
        t += dt;
    }
    /* 시간 측정 종료 */
    auto end_time = std::chrono::high_resolution_clock::now();
    /* 경과 시간 계산 (초 단위) */
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "total simulation time: " << elapsed.count() << " second" << std::endl;


    /*CSV 파일 저장*/
    for (int i = 0; i < total_vehicle_num; i++) {
        std::string filename = "../../make_csv/agent_" + std::to_string(i) + ".csv";
        histories[i].saveToCSV(filename);
        std::cout << "Agent " << i << " CSV saved!" << std::endl;
    }

    for (int i = 0; i < total_vehicle_num; i++) {
        error_logs[i].close();
        std::cout << "Agent " << i << " ZOH error log saved!" << std::endl;
    }




    /*스펙 JSON 저장-> CSV 파일을 만들기 위함 */
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