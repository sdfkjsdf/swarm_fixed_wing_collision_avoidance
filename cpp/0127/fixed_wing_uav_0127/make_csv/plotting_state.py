import pandas as pd
import matplotlib.pyplot as plt
import json

# Spec 읽기
with open('make_csv/aircraft_spec.json', 'r') as f:
    spec = json.load(f)

# CSV 읽기 (새 세로 형식)
df = pd.read_csv('make_csv/agent_3.csv')
time = df['time']

# 첫 번째 창: NED Position
fig1 = plt.figure(figsize=(12, 8))
fig1.suptitle('NED Position', fontsize=16)

plt.subplot(3, 1, 1)
plt.plot(time, df['north'], label='North', color='black')
plt.xlabel('Time (s)')
plt.ylabel('North (m)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, df['east'], label='East', color='black')
plt.xlabel('Time (s)')
plt.ylabel('East (m)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, df['down'], label='Down', color='black')
plt.xlabel('Time (s)')
plt.ylabel('Down (m)')
plt.legend()
plt.grid()

plt.tight_layout()

# 두 번째 창: Attitude and Speed
fig2 = plt.figure(figsize=(12, 8))
fig2.suptitle('Attitude and Speed', fontsize=16)

plt.subplot(2, 2, 1)
plt.plot(time, df['roll'], label='Roll', color='black')
plt.axhline(y=spec['max_roll'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_roll'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Roll (deg)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 2)
plt.plot(time, df['pitch'], label='Pitch', color='black')
plt.axhline(y=spec['max_pitch'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_pitch'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Pitch (deg)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 3)
plt.plot(time, df['yaw'], label='Yaw', color='black')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (deg)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 4)
plt.plot(time, df['speed'], label='Speed', color='black')
plt.axhline(y=spec['max_speed'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_speed'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend()
plt.grid()

plt.tight_layout()

# 세 번째 창: Angular Rates and Thrust-to-weight
fig3 = plt.figure(figsize=(12, 8))
fig3.suptitle('Angular Rates and Thrust-to-weight', fontsize=16)

plt.subplot(2, 2, 1)
plt.plot(time, df['p'], label='p (roll rate)', color='black')
plt.axhline(y=spec['max_p'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_p'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('p (degree/s)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 2)
plt.plot(time, df['q'], label='q (pitch rate)', color='black')
plt.axhline(y=spec['max_q'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_q'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('q (degree/s)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 3)
plt.plot(time, df['r'], label='r (yaw rate)', color='black')
plt.axhline(y=spec['max_r'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_r'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('r (degree/s)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 4)
plt.plot(time, df['at'], label='Thrust-to-weight', color='black')
plt.axhline(y=spec['max_at'], color='blue', linestyle='--', label='Max', alpha=0.7)
plt.axhline(y=spec['min_at'], color='red', linestyle='--', label='Min', alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('at (m/s^2)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()