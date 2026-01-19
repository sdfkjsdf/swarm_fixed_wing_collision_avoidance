#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <ctime>

int main() {
    const int total_vehicle_num = 5;
    const double kPi = 3.14159265358979323846;
    
    const std::string output_file = "../../build_initial_condition/initial_conditions.txt";
    
    // Random seed
    std::random_device rd;
    unsigned int seed = rd();
    std::mt19937 gen(seed);
    
    // Distribution settings
    std::uniform_real_distribution<> north_dist(-1000, 1000);
    std::uniform_real_distribution<> east_dist(-1000, 1000);
    std::uniform_real_distribution<> down_dist(-10, 10); 
    std::uniform_real_distribution<> yaw_dist(-kPi, kPi);
    std::uniform_real_distribution<> speed_dist(22, 30);
    
    // Create file
    std::ofstream file(output_file);
    if (!file.is_open()) {
        std::cerr << "Failed to create file: " << output_file << std::endl;
        return 1;
    }
    
    // Header
    std::time_t now = std::time(nullptr);
    file << "# Generated: " << std::ctime(&now);
    file << "# Seed: " << seed << std::endl;
    file << "# Agent_ID, North, East, Down, Yaw(rad), Speed\n";
    
    // Generate initial conditions
    for (int i = 0; i < total_vehicle_num; i++) {
        file << i << ", " 
             << north_dist(gen) << ", " 
             << east_dist(gen) << ", " 
             << down_dist(gen) << ", "   // ✅ 수정
             << yaw_dist(gen) << ", " 
             << speed_dist(gen) << "\n";
    }
    
    file.close();
    
    std::cout << "Initial conditions generated successfully!" << std::endl;
    std::cout << "Seed: " << seed << std::endl;
    std::cout << "File: " << output_file << std::endl;
    
    return 0;
}