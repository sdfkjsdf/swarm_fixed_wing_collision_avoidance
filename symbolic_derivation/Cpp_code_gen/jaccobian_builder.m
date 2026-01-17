% 지금 C++에 꽂아 넣을 Jacobian 코드 생성 (닫힌루프 체인룰 포함)

clear; clc;

%% 1) Symbolic variables
% state x = [n,e,d,roll,pitch,yaw,v,p,q,r,at]
syms n e d roll pitch yaw v p q r at real
state = [n,e,d,roll,pitch,yaw,v,p,q,r,at];

% in2: "setpoint" (너 C++에서 쓰는 p_setpoint, q_setpoint, ...)
syms p_setpoint q_setpoint r_setpoint at_setpoint real
set_points = [p_setpoint, q_setpoint, r_setpoint, at_setpoint];

% spec: 모델/제한 파라미터
syms g k_acc p_min p_max q_min q_max r_min r_max at_min at_max t_p t_q t_r t_a real
spec = [g,k_acc,p_min,p_max,q_min,q_max,r_min,r_max,at_min,at_max,t_p,t_q,t_r,t_a];

% gains: setpoint 추종 이득(너 C++의 k_*_sp)
syms k_p_sp k_q_sp k_r_sp k_at_sp real

% smoothness
syms k_smooth real

% (선택) 심볼릭 단순화 도움
assumeAlso(k_smooth > 0);
assumeAlso([t_p t_q t_r t_a] > 0);

%% 2) Trig helpers
c_p = cos(pitch); s_p = sin(pitch);
c_y = cos(yaw);   s_y = sin(yaw);
c_r = cos(roll);  s_r = sin(roll);
tan_p = tan(pitch);

%% 3) 너 C++ 구조 그대로: dot_*_sp -> *_cmd_rar
dot_p_sp  = k_p_sp  * (p_setpoint  - p);
dot_q_sp  = k_q_sp  * (q_setpoint  - q);
dot_r_sp  = k_r_sp  * (r_setpoint  - r);
dot_at_sp = k_at_sp * (at_setpoint - at);

p_cmd_rar  = p  + t_p * dot_p_sp;
q_cmd_rar  = q  + t_q * dot_q_sp;
r_cmd_rar  = r  + t_r * dot_r_sp;
at_cmd_rar = at + t_a * dot_at_sp;

%% 4) Final saturation (가능하면 min/max 모두 반영: 비대칭도 안전)
u1_final = subs_asym_sat(p_cmd_rar,  p_min,  p_max,  k_smooth);
u2_final = subs_asym_sat(q_cmd_rar,  q_min,  q_max,  k_smooth);
u3_final = subs_asym_sat(r_cmd_rar,  r_min,  r_max,  k_smooth);
u4_final = subs_asym_sat(at_cmd_rar, at_min, at_max, k_smooth);   % at_min=0이면 그대로 0

%% 5) Dynamics: dx = f(x) + g(x)u
fx = [
    v * c_p * c_y;
    v * c_p * s_y;
    -v * s_p;
    p + (q*s_r*tan_p) + (r*c_r*tan_p);
    (q*c_r) - (r*s_r);
    ((q*s_r) + (r*c_r)) / c_p;
    -g * s_p + at - (k_acc * (v*v));
    (-1 / t_p) * p;
    (-1 / t_q) * q;
    (-1 / t_r) * r;
    (-1 / t_a) * at
];

gx = sym(zeros(11,4));
gx(8,1)  = 1 / t_p;
gx(9,2)  = 1 / t_q;
gx(10,3) = 1 / t_r;
gx(11,4) = 1 / t_a;

dx_cl = fx + gx * [u1_final; u2_final; u3_final; u4_final];

%% 6) Jacobian wrt state (A = d(dx)/d(state))
fprintf('자코비안(A) 계산 중...\n');
J_sym = jacobian(dx_cl, state);

% (선택) setpoint에 대한 Jacobian도 같이 뽑고 싶으면:
% fprintf('자코비안(Bsp) 계산 중...\n');
% J_sp = jacobian(dx_cl, set_points);

%% 7) Export function
fprintf('함수 파일 생성 중...\n');
matlabFunction(J_sym, 'File', 'propagation_fcl_jacobian.m', ...
    'Vars', {state, set_points, spec, [k_p_sp k_q_sp k_r_sp k_at_sp], k_smooth}, ...
    'Optimize', true);

% Bsp까지 같이 내보내려면 위 matlabFunction을 2개 만들거나,
% outputs를 {J_sym, J_sp}로 묶어서 한 파일로도 가능해.

fprintf('완료! propagation_fcl_jacobian.m 생성됨.\n');
