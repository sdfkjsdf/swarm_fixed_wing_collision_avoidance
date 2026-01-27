#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H

#include <fstream>
#include <string>

inline void save_simulation_config(const std::string& filepath, 
                                   bool leader_follower_mode,
                                   double wingman_distance,
                                   double right_angle,
                                   double left_angle) 
{
    std::ofstream file(filepath);
    file << "{\n";
    file << "  \"leader_follower_mode\": " << (leader_follower_mode ? "true" : "false") << ",\n";
    file << "  \"wingman_distance\": " << wingman_distance << ",\n";
    file << "  \"right_angle_deg\": " << right_angle * 180.0 / 3.14159265358979 << ",\n";
    file << "  \"left_angle_deg\": " << left_angle * 180.0 / 3.14159265358979 << "\n";
    file << "}\n";
    file.close();
}

#endif // SIMULATION_CONFIG_H