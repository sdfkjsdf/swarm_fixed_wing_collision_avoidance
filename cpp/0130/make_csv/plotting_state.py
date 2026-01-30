import pandas as pd
import matplotlib.pyplot as plt
import json

# Spec 읽기
with open('make_csv/aircraft_spec.json', 'r') as f:
    spec = json.load(f)

# 에이전트 설정
num_agents = 5
colors = ['blue', 'orange', 'green', 'red', 'purple']

# 모든 에이전트 데이터 로드
dfs = []
for i in range(num_agents):
    df = pd.read_csv(f'make_csv/agent_{i}.csv')
    dfs.append(df)

time = dfs[0]['time']

# 첫 번째 창: NED Position
fig1 = plt.figure(figsize=(12, 8))
fig1.suptitle('NED Position (All Agents)', fontsize=16)

plt.subplot(3, 1, 1)
for i, df in enumerate(dfs):
    plt.plot(time, df['north'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('North (m)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
for i, df in enumerate(dfs):
    plt.plot(time, df['east'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('East (m)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
for i, df in enumerate(dfs):
    plt.plot(time, df['down'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Down (m)')
plt.legend()
plt.grid()

plt.tight_layout()

# 두 번째 창: Attitude and Speed
fig2 = plt.figure(figsize=(12, 8))
fig2.suptitle('Attitude and Speed (All Agents)', fontsize=16)

plt.subplot(2, 2, 1)
for i, df in enumerate(dfs):
    plt.plot(time, df['roll'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_roll'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_roll'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Roll (deg)')
plt.legend(fontsize=7)
plt.grid()

plt.subplot(2, 2, 2)
for i, df in enumerate(dfs):
    plt.plot(time, df['pitch'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_pitch'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_pitch'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Pitch (deg)')
plt.legend(fontsize=7)
plt.grid()

plt.subplot(2, 2, 3)
for i, df in enumerate(dfs):
    plt.plot(time, df['yaw'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Yaw (deg)')
plt.legend(fontsize=7)
plt.grid()

plt.subplot(2, 2, 4)
for i, df in enumerate(dfs):
    plt.plot(time, df['speed'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_speed'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_speed'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend(fontsize=7)
plt.grid()

plt.tight_layout()

# 세 번째 창: Angular Rates and Thrust-to-weight
fig3 = plt.figure(figsize=(12, 8))
fig3.suptitle('Angular Rates and Thrust-to-weight (All Agents)', fontsize=16)

plt.subplot(2, 2, 1)
for i, df in enumerate(dfs):
    plt.plot(time, df['p'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_p'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_p'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('p (degree/s)')
plt.legend(fontsize=7)
plt.grid()

plt.subplot(2, 2, 2)
for i, df in enumerate(dfs):
    plt.plot(time, df['q'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_q'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_q'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('q (degree/s)')
plt.legend(fontsize=7)
plt.grid()

plt.subplot(2, 2, 3)
for i, df in enumerate(dfs):
    plt.plot(time, df['r'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_r'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_r'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('r (degree/s)')
plt.legend(fontsize=7)
plt.grid()

plt.subplot(2, 2, 4)
for i, df in enumerate(dfs):
    plt.plot(time, df['at'], label=f'Agent {i}', color=colors[i], alpha=0.8)
plt.axhline(y=spec['max_at'], color='black', linestyle='--', label='Max', alpha=0.5)
plt.axhline(y=spec['min_at'], color='black', linestyle=':', label='Min', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('at (m/s^2)')
plt.legend(fontsize=7)
plt.grid()

plt.tight_layout()

# 네 번째 창: 2D 궤적 (추가)
fig4 = plt.figure(figsize=(10, 8))
fig4.suptitle('2D Trajectory (North-East)', fontsize=16)

for i, df in enumerate(dfs):
    plt.plot(df['east'], df['north'], label=f'Agent {i}', color=colors[i], alpha=0.8)
    plt.scatter(df['east'].iloc[0], df['north'].iloc[0], color=colors[i], marker='o', s=100)  # 시작점
    plt.scatter(df['east'].iloc[-1], df['north'].iloc[-1], color=colors[i], marker='x', s=100)  # 끝점

plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.legend()
plt.grid()
plt.axis('equal')
plt.tight_layout()

plt.show()