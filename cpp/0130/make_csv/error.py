import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

num_agents = 5
colors = ['blue', 'orange', 'green', 'red', 'purple']

# 컬럼명 정의
columns = ['time', 'error_n', 'error_e', 'error_d', 
           'error_roll', 'error_pitch', 'error_yaw',
           'error_speed', 'error_p', 'error_q', 'error_r', 'error_at']

# 데이터 로드
dfs = []
for i in range(num_agents):
    df = pd.read_csv(f'make_csv/zoh_error_agent_{i}.csv')
    dfs.append(df)

# ========== 전체 상태 오차 (11개) 계산 ==========
for i, df in enumerate(dfs):
    # 11개 상태에 대한 전체 오차 (L2 norm)
    state_errors = df[['error_n', 'error_e', 'error_d', 
                       'error_roll', 'error_pitch', 'error_yaw',
                       'error_speed', 'error_p', 'error_q', 'error_r', 'error_at']]
    df['total_error'] = np.sqrt((state_errors ** 2).sum(axis=1))

# ========== 플롯 ==========
fig, axes = plt.subplots(4, 3, figsize=(16, 12))
fig.suptitle('ZOH vs RK4 One-Step Error (All 11 States)', fontsize=16)

state_names = ['North (m)', 'East (m)', 'Down (m)', 
               'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)',
               'Speed (m/s)', 'p (rad/s)', 'q (rad/s)', 'r (rad/s)', 'at (m/s²)']

error_cols = ['error_n', 'error_e', 'error_d', 
              'error_roll', 'error_pitch', 'error_yaw',
              'error_speed', 'error_p', 'error_q', 'error_r', 'error_at']

# 11개 상태 개별 플롯
for idx, (col, name) in enumerate(zip(error_cols, state_names)):
    ax = axes[idx // 3, idx % 3]
    for i, df in enumerate(dfs):
        ax.plot(df['time'], df[col], label=f'Agent {i}', color=colors[i], alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(f'Error {name}')
    ax.set_title(name)
    ax.grid(True)
    ax.legend(fontsize=7)

# 마지막 칸: 전체 오차
ax = axes[3, 2]
for i, df in enumerate(dfs):
    ax.plot(df['time'], df['total_error'], label=f'Agent {i}', color=colors[i], alpha=0.7)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Total Error')
ax.set_title('Total State Error (L2 norm)')
ax.grid(True)
ax.legend(fontsize=7)

plt.tight_layout()
plt.savefig('zoh_error_all_states.png', dpi=150)

# ========== 통계 출력 ==========
print("\n" + "="*70)
print("ZOH vs RK4 One-Step Error Statistics (All 11 States)")
print("="*70)

for i, df in enumerate(dfs):
    print(f"\n{'='*30} Agent {i} {'='*30}")
    for col, name in zip(error_cols, state_names):
        max_err = df[col].abs().max()
        mean_err = df[col].abs().mean()
        print(f"  {name:15s} | Max: {max_err:12.6e} | Mean: {mean_err:12.6e}")
    print(f"  {'Total L2':15s} | Max: {df['total_error'].max():12.6e} | Mean: {df['total_error'].mean():12.6e}")

plt.show()