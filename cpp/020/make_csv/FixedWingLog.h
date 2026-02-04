#ifndef FIXEDWINGLOG_H
#define FIXEDWINGLOG_H

#include <matrix/math.hpp>
#include <vector>
#include <string>
#include <fstream>


struct FixedWingData_t {
    double time;
    double north;
    double east;
    double down;
    double roll;
    double pitch;
    double yaw;
    double speed;
    double p;
    double q;
    double r;
    double at;

};

class FixedWingLog{

    public:
        FixedWingLog(int id) : m_agent_id(id){}

        void addData(double t, const matrix::Vector<double, 11>& state) {
                        FixedWingData_t d;
                        constexpr double rad2deg = 180.0 / 3.14159265358979323846;

                        d.time = t;
                        d.north = state(0);
                        d.east  = state(1);
                        d.down  = state(2);
                        d.roll  = state(3) * rad2deg;
                        d.pitch = state(4) * rad2deg;
                        d.yaw   = state(5) * rad2deg;
                        d.speed = state(6);
                        d.p     = state(7) * rad2deg;
                        d.q     = state(8) * rad2deg;
                        d.r     = state(9) * rad2deg;
                        d.at    = state(10);

                        m_data_history.push_back(d);
                    }

        void saveToCSV(const std::string& filename) {
            std::ofstream csv(filename);
            
            // 헤더 (세로 형식 - 표준)
            csv << "time,north,east,down,roll,pitch,yaw,speed,p,q,r,at\n";
            
            // 데이터 (한 행 = 한 시점)
            for(const auto& d : m_data_history) {
                csv << d.time << ","
                    << d.north << ","
                    << d.east << ","
                    << d.down << ","
                    << d.roll << ","
                    << d.pitch << ","
                    << d.yaw << ","
                    << d.speed << ","
                    << d.p << ","
                    << d.q << ","
                    << d.r << ","
                    << d.at << "\n";
            }
            
            csv.close();
        }
    
    int getAgentId() const { return m_agent_id; }


    private:
        int m_agent_id=0;
        std::vector<FixedWingData_t> m_data_history{};

};

#endif //  FIXEDWINGDATA_H