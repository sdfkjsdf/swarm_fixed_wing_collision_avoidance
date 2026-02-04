import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from datetime import datetime
from itertools import combinations

class XQ58_Valkyrie:
    """XQ-58A Valkyrie 스타일 스텔스 드론 3D 모델"""
    
    def __init__(self, scale=1.0):
        self.scale = scale
        
        L = 8.8 * scale
        W = 6.7 * scale
        H = 0.8 * scale
        
        nose_len = 3.0 * scale
        body_len = 4.0 * scale
        tail_len = 1.8 * scale
        body_width = 1.2 * scale
        body_height = H
        
        wing_root_chord = 2.5 * scale
        wing_tip_chord = 0.8 * scale
        wing_span_half = W / 2
        wing_sweep = 2.5 * scale
        wing_z = -0.1 * scale
        
        vtail_span = 1.2 * scale
        vtail_chord = 1.0 * scale
        vtail_angle = 40
        vtail_sweep = 0.5 * scale
        
        self.verts_fuselage = np.array([
            [L, 0, 0],
            [L - nose_len, body_width/2, -body_height/3],
            [L - nose_len, -body_width/2, -body_height/3],
            [L - nose_len, body_width/3, body_height/2],
            [L - nose_len, -body_width/3, body_height/2],
            [L - nose_len - body_len, body_width/2, -body_height/3],
            [L - nose_len - body_len, -body_width/2, -body_height/3],
            [L - nose_len - body_len, body_width/3, body_height/2],
            [L - nose_len - body_len, -body_width/3, body_height/2],
            [0, body_width/4, -body_height/4],
            [0, -body_width/4, -body_height/4],
            [0, body_width/6, body_height/4],
            [0, -body_width/6, body_height/4],
        ])
        
        self.faces_fuselage = [
            [0, 1, 2], [0, 1, 3], [0, 2, 4], [0, 3, 4],
            [1, 2, 6, 5], [3, 4, 8, 7], [1, 3, 7, 5], [2, 4, 8, 6],
            [5, 6, 10, 9], [7, 8, 12, 11], [5, 7, 11, 9], [6, 8, 12, 10],
            [9, 10, 12, 11],
        ]
        
        wing_root_x = L - nose_len - 0.5 * scale
        
        self.verts_wing_right = np.array([
            [wing_root_x, body_width/2, wing_z],
            [wing_root_x - wing_root_chord, body_width/2, wing_z],
            [wing_root_x - wing_sweep, wing_span_half, wing_z],
            [wing_root_x - wing_sweep - wing_tip_chord, wing_span_half, wing_z],
        ])
        
        self.verts_wing_left = np.array([
            [wing_root_x, -body_width/2, wing_z],
            [wing_root_x - wing_root_chord, -body_width/2, wing_z],
            [wing_root_x - wing_sweep, -wing_span_half, wing_z],
            [wing_root_x - wing_sweep - wing_tip_chord, -wing_span_half, wing_z],
        ])
        
        self.faces_wing = [[0, 1, 3, 2]]
        
        vtail_root_x = tail_len * 0.3
        vtail_tip_z = body_height/4 + vtail_span * np.sin(np.deg2rad(vtail_angle))
        vtail_tip_y = vtail_span * np.cos(np.deg2rad(vtail_angle))
        
        self.verts_vtail_right = np.array([
            [vtail_root_x + vtail_chord, body_width/4, body_height/4],
            [vtail_root_x, body_width/4, body_height/4],
            [vtail_root_x + vtail_chord - vtail_sweep, vtail_tip_y, vtail_tip_z],
            [vtail_root_x - vtail_sweep, vtail_tip_y, vtail_tip_z],
        ])
        
        self.verts_vtail_left = np.array([
            [vtail_root_x + vtail_chord, -body_width/4, body_height/4],
            [vtail_root_x, -body_width/4, body_height/4],
            [vtail_root_x + vtail_chord - vtail_sweep, -vtail_tip_y, vtail_tip_z],
            [vtail_root_x - vtail_sweep, -vtail_tip_y, vtail_tip_z],
        ])
        
        self.faces_vtail = [[0, 1, 3, 2]]
        
        self.color_fuselage = [0.35, 0.35, 0.38]
        self.color_wing = [0.4, 0.4, 0.43]
        self.color_vtail = [0.3, 0.3, 0.33]
        
        self.outline_2d = self._create_2d_outline(L, W, nose_len, body_width,
                                                   wing_root_x, wing_root_chord,
                                                   wing_sweep, wing_tip_chord, wing_span_half)
    
    def _create_2d_outline(self, L, W, nose_len, body_width,
                           wing_root_x, wing_root_chord, wing_sweep, wing_tip_chord, wing_span_half):
        outline = np.array([
            [L, 0],
            [L - nose_len, body_width/2],
            [wing_root_x, body_width/2],
            [wing_root_x - wing_sweep, wing_span_half],
            [wing_root_x - wing_sweep - wing_tip_chord, wing_span_half],
            [wing_root_x - wing_root_chord, body_width/2],
            [0, body_width/4],
            [0, -body_width/4],
            [wing_root_x - wing_root_chord, -body_width/2],
            [wing_root_x - wing_sweep - wing_tip_chord, -wing_span_half],
            [wing_root_x - wing_sweep, -wing_span_half],
            [wing_root_x, -body_width/2],
            [L - nose_len, -body_width/2],
            [L, 0],
        ])
        return outline
    
    def rotation_matrix(self, roll_rad, pitch_rad, yaw_rad):
        cr, sr = np.cos(roll_rad), np.sin(roll_rad)
        cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
        cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
        
        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
        return R_z @ R_y @ R_x
    
    def draw(self, ax, position, roll_deg, pitch_deg, yaw_deg, alpha=0.9):
        roll_rad = np.deg2rad(roll_deg)
        pitch_rad = np.deg2rad(pitch_deg)
        yaw_rad = np.deg2rad(yaw_deg)
        
        R = self.rotation_matrix(roll_rad, pitch_rad, yaw_rad)
        
        def transform(verts):
            return (R @ verts.T).T + position
        
        verts_f = transform(self.verts_fuselage)
        for face in self.faces_fuselage:
            poly = Poly3DCollection([verts_f[face]],
                                    facecolors=self.color_fuselage,
                                    edgecolors=self.color_fuselage,
                                    linewidths=0.1, alpha=alpha)
            ax.add_collection3d(poly)
        
        verts_wr = transform(self.verts_wing_right)
        for face in self.faces_wing:
            poly = Poly3DCollection([verts_wr[face]],
                                    facecolors=self.color_wing,
                                    edgecolors=self.color_wing,
                                    linewidths=0.1, alpha=alpha)
            ax.add_collection3d(poly)
        
        verts_wl = transform(self.verts_wing_left)
        for face in self.faces_wing:
            poly = Poly3DCollection([verts_wl[face]],
                                    facecolors=self.color_wing,
                                    edgecolors=self.color_wing,
                                    linewidths=0.1, alpha=alpha)
            ax.add_collection3d(poly)
        
        verts_vtr = transform(self.verts_vtail_right)
        for face in self.faces_vtail:
            poly = Poly3DCollection([verts_vtr[face]],
                                    facecolors=self.color_vtail,
                                    edgecolors=self.color_vtail,
                                    linewidths=0.1, alpha=alpha)
            ax.add_collection3d(poly)
        
        verts_vtl = transform(self.verts_vtail_left)
        for face in self.faces_vtail:
            poly = Poly3DCollection([verts_vtl[face]],
                                    facecolors=self.color_vtail,
                                    edgecolors=self.color_vtail,
                                    linewidths=0.1, alpha=alpha)
            ax.add_collection3d(poly)
    
    def draw_2d(self, ax, north, east, yaw_deg, color_idx=0, alpha=0.85):
        yaw_rad = np.deg2rad(yaw_deg)
        
        cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
        R_2d = np.array([[cy, -sy], [sy, cy]])
        
        outline_rotated = (R_2d @ self.outline_2d.T).T + np.array([north, east])
        
        colors = [
            [0.3, 0.3, 0.35],
            [0.4, 0.25, 0.25],
            [0.25, 0.35, 0.25],
            [0.25, 0.3, 0.4],
            [0.35, 0.3, 0.25],
        ]
        fill_color = colors[color_idx % len(colors)]
        
        ax.fill(outline_rotated[:, 0], outline_rotated[:, 1],
                color=fill_color, alpha=alpha, zorder=5)  # zorder 높임
        ax.plot(outline_rotated[:, 0], outline_rotated[:, 1],
                color=[0.1, 0.1, 0.1], linewidth=1.0, alpha=alpha, zorder=6)  # zorder 높임


def load_csv(filename):
    """CSV 로드"""
    return pd.read_csv(filename)


def find_min_distance_pair_with_hysteresis(positions, prev_pair, threshold=1.0):
    """Find the pair with minimum distance, with hysteresis to avoid flickering"""
    n = positions.shape[1]
    
    # 모든 쌍의 거리 계산
    distances = {}
    for i, j in combinations(range(n), 2):
        dist = np.linalg.norm(positions[:, i] - positions[:, j])
        distances[(i, j)] = dist
    
    # 정렬하여 최소, 2번째 최소 찾기
    sorted_pairs = sorted(distances.items(), key=lambda x: x[1])
    min_pair, min_dist = sorted_pairs[0]
    second_pair, second_dist = sorted_pairs[1]
    
    # 이전 쌍이 있고, 최소와 2번째 차이가 threshold 이하면 이전 쌍 유지
    if prev_pair is not None:
        prev_dist = distances.get(prev_pair, np.inf)
        
        # 이전 쌍의 거리와 현재 최소 거리 차이가 threshold 이하면 유지
        if abs(prev_dist - min_dist) < threshold:
            return prev_pair, prev_dist
    
    return min_pair, min_dist


def create_formation_animation(csv_folder='make_csv'):
    """5대 UAV 편대비행 애니메이션 생성"""
    
    print('🚀 XQ-58A Valkyrie 스타일 3D 비디오 생성 시작...')
    
    # CSV 데이터 읽기
    agents = []
    n = 5
    for i in range(n):
        df = load_csv(f'{csv_folder}/agent_{i}.csv')
        agents.append(df)
        print(f"  Agent {i} loaded: {len(df)} timesteps")
    
    # 프레임 설정
    plot_skip = 240
    tf = len(agents[0])
    plot_indices = list(range(0, tf, plot_skip))
    num_plot_frames = len(plot_indices)
    
    print(f'📊 총 {tf}개 타임스텝 중 {num_plot_frames}개 프레임 생성 (skip={plot_skip})')
    
    # 비디오 파일명
    day = datetime.now().strftime('%Y%m%d_%H%M%S')
    video_filename = f'{day}_XQ58A_Flocking.mp4'
    
    # Figure 설정
    fig = plt.figure(figsize=(18, 8))
    fig.patch.set_facecolor('white')
    
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.set_facecolor([0.9, 0.95, 1.0])
    
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_facecolor([0.95, 0.97, 0.95])
    
    plt.subplots_adjust(left=0.05, right=0.95, wspace=0.25)
    
    model_scale_3d = 3.0
    model_scale_2d = 1.5
    
    uav_models_3d = [XQ58_Valkyrie(scale=model_scale_3d) for _ in range(n)]
    uav_models_2d = [XQ58_Valkyrie(scale=model_scale_2d) for _ in range(n)]
    L_base = 8.8 * model_scale_3d
    
    # ✅ 수정: 제목에서 최소거리 제거, 시간만 표시
    text_annotation = fig.text(0.5, 0.96, '', fontsize=11, fontweight='bold',
                               ha='center', va='top',
                               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9))
    
    # ✅ 히스테리시스를 위한 이전 쌍 저장
    prev_pair = [None]  # mutable object for closure
    
    def update(frame_idx):
        k = plot_indices[frame_idx]
        
        # 데이터 준비
        current_positions_ned = np.array([
            [agent['north'].iloc[k], agent['east'].iloc[k], agent['down'].iloc[k]]
            for agent in agents
        ]).T
        
        current_attitudes_deg = np.array([
            [agent['roll'].iloc[k], agent['pitch'].iloc[k], agent['yaw'].iloc[k]]
            for agent in agents
        ]).T
        
        # NED → XYZ 변환
        current_positions_xyz = current_positions_ned.copy()
        current_positions_xyz[2, :] = -current_positions_ned[2, :]
        
        # 축 범위 계산
        center = np.mean(current_positions_xyz, axis=1)
        max_dev = np.max(np.abs(current_positions_xyz - center[:, np.newaxis]))
        plot_range = max(max_dev * 1.2 + L_base, L_base * 1.5)
        z_range = plot_range * 0.3
        
        # ✅ 히스테리시스 적용한 최소 거리 쌍 찾기
        min_pair, min_d = find_min_distance_pair_with_hysteresis(
            current_positions_ned, prev_pair[0], threshold=1.0)
        prev_pair[0] = min_pair  # 다음 프레임을 위해 저장
        
        ax1.clear()
        ax2.clear()
        
        # 3D 뷰 설정
        ax1.set_xlim([center[0] - plot_range, center[0] + plot_range])
        ax1.set_ylim([center[1] - plot_range, center[1] + plot_range])
        ax1.set_zlim([center[2] - z_range, center[2] + z_range])
        ax1.set_xlabel('North [m]', fontsize=10, labelpad=10)
        ax1.set_ylabel('East [m]', fontsize=10, labelpad=10)
        ax1.set_zlabel('Up [m]', fontsize=10, labelpad=10)
        ax1.set_title('3D Perspective View', fontsize=12, fontweight='bold')
        ax1.view_init(elev=30, azim=135)
        ax1.set_facecolor([0.9, 0.95, 1.0])
        
        # 2D 뷰 설정
        ax2.set_xlim([center[0] - plot_range, center[0] + plot_range])
        ax2.set_ylim([center[1] - plot_range, center[1] + plot_range])
        ax2.set_xlabel('North [m]', fontsize=10)
        ax2.set_ylabel('East [m]', fontsize=10)
        ax2.set_title('Top View (Bird\'s Eye)', fontsize=12, fontweight='bold')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.set_facecolor([0.95, 0.97, 0.95])
        
        # ✅ 수정: 빨간 선 먼저 그리기 (zorder=1, 비행기보다 뒤)
        i1, i2 = min_pair
        pos1 = current_positions_ned[:, i1]
        pos2 = current_positions_ned[:, i2]
        
        ax2.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 
                 'r-', linewidth=1.0, zorder=1, alpha=0.7)  # 선 두께 줄임, zorder 낮춤
        
        # UAV 그리기 (빨간 선 위에)
        for j in range(n):
            pos_ned = current_positions_ned[:, j]
            att_deg = current_attitudes_deg[:, j]
            pos_xyz = np.array([pos_ned[0], pos_ned[1], -pos_ned[2]])
            
            uav_models_3d[j].draw(ax1, pos_xyz, att_deg[0], att_deg[1], att_deg[2])
            uav_models_2d[j].draw_2d(ax2, pos_ned[0], pos_ned[1], att_deg[2], color_idx=j)
            
            ax2.text(pos_ned[0], pos_ned[1] + 8.8 * model_scale_2d * 0.8, f'{j}',
                    fontsize=9, ha='center', va='bottom', fontweight='bold',
                    color=[0.2, 0.2, 0.2], zorder=10)
        
        # ✅ 수정: 오른쪽 plot 우측 상단에 최소거리 표시
        ax2.text(0.98, 0.98, f'Min Dist: {min_d:.1f}m\n(Agent {i1} ↔ {i2})', 
                 transform=ax2.transAxes,
                 fontsize=10, ha='right', va='top',
                 fontweight='bold', color='red',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9))
        
        # ✅ 수정: 상단 제목은 시간만 표시
        text_annotation.set_text(f'Time: {agents[0]["time"].iloc[k]:.1f}s')
        
        return []
    
    print('🎬 프레임 렌더링 중...')
    anim = FuncAnimation(fig, update, frames=num_plot_frames,
                        interval=100, blit=False, repeat=True)
    
    return fig, anim, video_filename, num_plot_frames


if __name__ == '__main__':
    print("=" * 50)
    print("XQ-58A Valkyrie Formation Flight Visualization")
    print("=" * 50)
    
    fig, anim, video_filename, total_frames = create_formation_animation()
    
    def progress_callback(current_frame, total_frames):
        percent = (current_frame + 1) / total_frames * 100
        bar_len = 40
        filled = int(bar_len * (current_frame + 1) / total_frames)
        bar = '█' * filled + '░' * (bar_len - filled)
        print(f'\r💾 저장 중: [{bar}] {percent:5.1f}% ({current_frame + 1}/{total_frames})', end='', flush=True)
    
    print(f'💾 비디오 저장 시작: {video_filename}')
    anim.save(video_filename, writer='ffmpeg', fps=10, dpi=100, bitrate=2000,
              progress_callback=lambda i, n: progress_callback(i, total_frames))
    
    print(f'\n✅ 완료: {video_filename}')
    plt.show()