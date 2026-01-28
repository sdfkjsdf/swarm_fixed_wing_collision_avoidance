import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
import os
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def load_simulation_config():
    """C++ 시뮬레이션에서 저장한 설정 파일 로드"""
    config_path = os.path.join(SCRIPT_DIR, 'sim_config.json')
    
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = json.load(f)
        print(f"  Loaded config from {config_path}")
        return config
    else:
        print(f"  Warning: {config_path} not found, using defaults")
        return {
            "leader_follower_mode": False,
            "wingman_distance": 100.0,
            "right_angle_deg": 45.0,
            "left_angle_deg": -45.0
        }

def calculate_wingman_setposition(distance, angle_rad, ratio):
    """practice.cpp의 WingManPostion::calculate_wingman_setposition과 동일"""
    dx = ratio * distance * np.cos(angle_rad)
    dy = ratio * distance * np.sin(angle_rad)
    return np.array([dx, dy])

def get_desired_positions(config):
    """설정에서 각 wingman의 목표 위치 계산"""
    distance = config["wingman_distance"]
    right_angle = config["right_angle_deg"] * np.pi / 180
    left_angle = config["left_angle_deg"] * np.pi / 180
    
    return {
        0: np.array([0.0, 0.0]),  # Leader
        1: calculate_wingman_setposition(distance, right_angle, 1.0),
        2: calculate_wingman_setposition(distance, right_angle, 2.0),
        3: calculate_wingman_setposition(distance, left_angle, 1.0),
        4: calculate_wingman_setposition(distance, left_angle, 2.0),
    }

def load_all_agents(num_agents=5):
    """Load all agent CSV files"""
    print("  Loading CSV files...")
    agents = []
    for i in range(num_agents):
        filepath = os.path.join(SCRIPT_DIR, f'agent_{i}.csv')
        df = pd.read_csv(filepath)
        agents.append(df)
    return agents

def calculate_min_distance_fast(agents):
    """Vectorized minimum distance calculation"""
    num_agents = len(agents)
    positions = [agent[['north', 'east', 'down']].values for agent in agents]
    
    pairs = list(combinations(range(num_agents), 2))
    all_distances = np.zeros((len(agents[0]), len(pairs)))
    
    for idx, (i, j) in enumerate(pairs):
        diff = positions[i] - positions[j]
        all_distances[:, idx] = np.sqrt(np.sum(diff**2, axis=1))
    
    return np.min(all_distances, axis=1)

def calculate_position_std_fast(agents):
    """Vectorized position std calculation"""
    positions = np.array([agent[['north', 'east', 'down']].values for agent in agents])
    centroid = np.mean(positions, axis=0)
    distances = np.sqrt(np.sum((positions - centroid)**2, axis=2))
    return np.std(distances, axis=0)

def calculate_speed_std_fast(agents):
    """Vectorized speed std calculation"""
    speeds = np.array([agent['speed'].values for agent in agents])
    return np.std(speeds, axis=0)

def calculate_formation_error(agents, desired_positions):
    """리더-팔로워 모드: 각 wingman의 위치 추종 오차 계산"""
    num_timesteps = len(agents[0])
    num_wingmen = len(agents) - 1
    
    leader_north = agents[0]['north'].values
    leader_east = agents[0]['east'].values
    leader_yaw = agents[0]['yaw'].values * np.pi / 180
    
    wingman_errors = np.zeros((num_wingmen, num_timesteps))
    
    for i in range(1, len(agents)):
        wingman_north = agents[i]['north'].values
        wingman_east = agents[i]['east'].values
        
        rel_north = wingman_north - leader_north
        rel_east = wingman_east - leader_east
        
        c_yaw = np.cos(leader_yaw)
        s_yaw = np.sin(leader_yaw)
        
        dx = c_yaw * rel_north + s_yaw * rel_east
        dy = -s_yaw * rel_north + c_yaw * rel_east
        
        desired = desired_positions[i]
        error_dx = dx - desired[0]
        error_dy = dy - desired[1]
        
        wingman_errors[i-1] = np.sqrt(error_dx**2 + error_dy**2)
    
    return wingman_errors

def main_flocking(agents, time):
    """플로킹 모드 분석"""
    print("Calculating metrics (Flocking Mode)...")
    min_distances = calculate_min_distance_fast(agents)
    position_stds = calculate_position_std_fast(agents)
    speed_stds = calculate_speed_std_fast(agents)
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Flocking Mode Analysis', fontsize=14, fontweight='bold')
    
    axes[0].plot(time, min_distances, 'b-', linewidth=1)
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Minimum Distance [m]')
    axes[0].set_title('Minimum Inter-Agent Distance')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([time[0], time[-1]])
    
    axes[1].plot(time, position_stds, 'g-', linewidth=1)
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Position Std Dev [m]')
    axes[1].set_title('Position Standard Deviation from Centroid')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim([time[0], time[-1]])
    
    axes[2].plot(time, speed_stds, 'r-', linewidth=1)
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Speed Std Dev [m/s]')
    axes[2].set_title('Speed Standard Deviation')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim([time[0], time[-1]])
    
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'formation_analysis.png'), dpi=150)
    plt.show()
    
    print("\n========== Summary Statistics (Flocking Mode) ==========")
    print(f"Minimum Distance: Min={min_distances.min():.2f}m, Max={min_distances.max():.2f}m")
    print(f"Position Std: Final={position_stds[-1]:.2f}m")
    print(f"Speed Std: Final={speed_stds[-1]:.2f}m/s")

def main_leader_follower(agents, time, desired_positions):
    """리더-팔로워 모드 분석"""
    print("Calculating metrics (Leader-Follower Mode)...")
    
    min_distances = calculate_min_distance_fast(agents)
    wingman_errors = calculate_formation_error(agents, desired_positions)
    speed_stds = calculate_speed_std_fast(agents)
    mean_formation_error = np.mean(wingman_errors, axis=0)
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Leader-Follower Mode Analysis', fontsize=14, fontweight='bold')
    
    axes[0].plot(time, min_distances, 'b-', linewidth=1)
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Minimum Distance [m]')
    axes[0].set_title('Minimum Inter-Agent Distance (Safety Check)')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([time[0], time[-1]])
    
    colors = ['orange', 'green', 'red', 'purple']
    for i in range(wingman_errors.shape[0]):
        axes[1].plot(time, wingman_errors[i], color=colors[i], linewidth=1, 
                     label=f'Wingman {i+1}', alpha=0.7)
    axes[1].plot(time, mean_formation_error, 'k-', linewidth=2, label='Mean Error')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Position Error [m]')
    axes[1].set_title('Formation Position Error (Distance from Desired Position)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim([time[0], time[-1]])
    
    axes[2].plot(time, speed_stds, 'r-', linewidth=1)
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Speed Std Dev [m/s]')
    axes[2].set_title('Speed Standard Deviation')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim([time[0], time[-1]])
    
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'formation_analysis.png'), dpi=150)
    plt.show()
    
    print("\n========== Summary Statistics (Leader-Follower Mode) ==========")
    print(f"Minimum Distance: Min={min_distances.min():.2f}m, Max={min_distances.max():.2f}m")
    print(f"\n[Formation Error - Final Values]")
    for i in range(wingman_errors.shape[0]):
        desired = desired_positions[i+1]
        print(f"  Wingman {i+1} (Target: dx={desired[0]:.1f}m, dy={desired[1]:.1f}m): "
              f"Error={wingman_errors[i, -1]:.2f}m")
    print(f"\n  Mean Formation Error: Final={mean_formation_error[-1]:.2f}m")
    print(f"Speed Std: Final={speed_stds[-1]:.2f}m/s")

def main():
    print("Loading simulation config...")
    config = load_simulation_config()
    
    print("Loading agent data...")
    agents = load_all_agents(5)
    time = agents[0]['time'].values
    
    if config["leader_follower_mode"]:
        print("\n*** Leader-Follower Mode (Auto-detected) ***\n")
        desired_positions = get_desired_positions(config)
        main_leader_follower(agents, time, desired_positions)
    else:
        print("\n*** Flocking Mode (Auto-detected) ***\n")
        main_flocking(agents, time)

if __name__ == "__main__":
    main()