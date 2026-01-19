import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

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
    
    # Extract position arrays (N x 3)
    positions = []
    for agent in agents:
        pos = agent[['north', 'east', 'down']].values
        positions.append(pos)
    
    # Calculate all pairwise distances
    pairs = list(combinations(range(num_agents), 2))
    all_distances = np.zeros((len(agents[0]), len(pairs)))
    
    for idx, (i, j) in enumerate(pairs):
        diff = positions[i] - positions[j]
        all_distances[:, idx] = np.sqrt(np.sum(diff**2, axis=1))
    
    return np.min(all_distances, axis=1)

def calculate_position_std_fast(agents):
    """Vectorized position std calculation"""
    # Stack all positions: (num_agents, num_timesteps, 3)
    positions = np.array([agent[['north', 'east', 'down']].values for agent in agents])
    
    # Centroid: (num_timesteps, 3)
    centroid = np.mean(positions, axis=0)
    
    # Distances from centroid: (num_agents, num_timesteps)
    distances = np.sqrt(np.sum((positions - centroid)**2, axis=2))
    
    # Std across agents: (num_timesteps,)
    return np.std(distances, axis=0)

def calculate_speed_std_fast(agents):
    """Vectorized speed std calculation"""
    speeds = np.array([agent['speed'].values for agent in agents])
    return np.std(speeds, axis=0)

def main():
    print("Loading agent data...")
    agents = load_all_agents(5)
    time = agents[0]['time'].values
    
    print("Calculating metrics (vectorized)...")
    min_distances = calculate_min_distance_fast(agents)
    position_stds = calculate_position_std_fast(agents)
    speed_stds = calculate_speed_std_fast(agents)
    
    print("Plotting...")
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Plot 1: Minimum distance
    axes[0].plot(time, min_distances, 'b-', linewidth=1)
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Minimum Distance [m]')
    axes[0].set_title('Minimum Inter-Agent Distance')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([time[0], time[-1]])
    
    # Plot 2: Position std
    axes[1].plot(time, position_stds, 'g-', linewidth=1)
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Position Std Dev [m]')
    axes[1].set_title('Position Standard Deviation from Centroid')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim([time[0], time[-1]])
    
    # Plot 3: Speed std
    axes[2].plot(time, speed_stds, 'r-', linewidth=1)
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Speed Std Dev [m/s]')
    axes[2].set_title('Speed Standard Deviation')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim([time[0], time[-1]])
    
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'formation_analysis.png'), dpi=150)
    plt.show()
    
    print("\n========== Summary Statistics ==========")
    print(f"Minimum Distance: Min={min_distances.min():.2f}m, Max={min_distances.max():.2f}m")
    print(f"Position Std: Final={position_stds[-1]:.2f}m")
    print(f"Speed Std: Final={speed_stds[-1]:.2f}m/s")

if __name__ == "__main__":
    main()