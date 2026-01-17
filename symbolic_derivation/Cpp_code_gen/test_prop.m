% 지금 올리는 코드는 1201일 기준으로 작성이 된것인다

    g=9.81 ;
    v_max=292 ;  % m/s
    v_min=80   ; % m/s
    
    pitch_max=deg2rad(25);
    pitch_min=deg2rad(-20);
    roll_max=deg2rad(40);
    roll_min=-deg2rad(40);
    
    roll_rate_max=deg2rad(90);
    roll_rate_min=-deg2rad(90);

  
    nz_max=2;
    nz_min=0;
    
    at_max=3.5; % 5~10 
    at_min=0; %
    roll_margin=deg2rad(3);
    pitch_margin=deg2rad(3);
    speed_margin=0.01;

% 시상수 정의
    t_v= 1.0 ; % 2.0~5.0
    t_p= 0.5 ; % 0.5~0.8
    t_z = 0.8 ; %0.5~1.0 
    t_yaw = 2.0 ; %1.0~2.0


  spec_constraint=[v_max;v_min;pitch_max;pitch_min;roll_max;roll_min;roll_rate_max;roll_rate_min;nz_max;nz_min;at_max;at_min];
  time_constant=[t_yaw;t_p;t_z;t_v];

  % v_max=spec_constraint(1,1);
  % v_min=spec_constraint(2,1);
  % pitch_max=spec_constraint(3,1);
  % pitch_min=spec_constraint(4,1);
  % roll_max=spec_constraint(5,1);
  % roll_min=spec_constraint(6,1);
  % roll_rate_max=spec_constraint(7,1);
  % roll_rate_min=spec_constraint(8,1);
  % nz_max=spec_constraint(9,1);
  % nz_min=spec_constraint(10,1);
  % at_max=spec_constraint(11,1);
  % at_min=spec_constraint(12,1);

  % t_yaw=time_constant(1,1);
  % t_p=time_constant(2,1);
  % t_z=time_constant(3,1);
  % t_v=time_constant(4,1);

  







%% 파라미터 정의
%% outer loop
k_h=0.8 ;%0.3~0.8  고도 제어 게인
k_v =0.5; % 0.2~0.5 속도 제어 게인 $$a_{t, cmd} = \underbrace{K_v (V_{sp} - V_T)}_{\text{1. 속도 오차 제어}} + \underbrace{1 \cdot g \sin(\theta)}_{\text{2. 중력(협조) 보상}}$$
k_yaw=1.0; % 0.5~1.0 경로(롤) 유도 게인

%% inner loop
k_pitch=9 ; % 9.0~25.0 , 피치 자세 게인 (P)
k_pitch_damp=7 ; % 4.2~7.0 피치 댐핑 게인 (D)
k_nz=3.0;  %3.0~5.0 $N_z$ 추종 게인
k_roll=5.0; %3.0~5.0 롤 자세 게인
k_P=15 ; % 10~15 
k_acc=0.01;


outer_loop_gain=[k_h;k_v;k_yaw];

% k_h=outer_loop_gain(1,1);
% k_v=outer_loop_gain(2,1);
% k_yaw=outer_loop_gain(3,1);

inner_loop_gain=[k_pitch;k_pitch_damp;k_nz;k_roll;k_P];

k_smooth=30; % 나중에 soft min 을 구현할 떄 쓰는 파라미터이기는 함
k_smooth_velocity=k_smooth/100;
% 
% k_pitch=inner_loop_gain(1,1);
% k_pitch_damp=inner_loop_gain(2,1);
% k_nz=inner_loop_gain(3,1);
% k_roll=inner_loop_gain(4,1);
% k_P=inner_loop_gain(5,1);
% 

%% Setpoint 선언 




%% 나중에 자코비안 지금 심볼릭 미분한 것을 넣을 때 파라미터 한꺼번에 넣으면 되는 것들 
%% in3 -> 지금 params을 넣으면 됨




in1 = [0; 0; 0; 0; 0; 0; 30 ; 0; 0; 0];

in2=[0,0,0];

in3 = [k_nz, k_roll, k_P, ...
          roll_rate_max, roll_rate_min, ...
          nz_max, nz_min, at_max, at_min, ...
          t_p, t_z, t_v, g,k_acc, k_smooth];

J = propagation_fcl_jacobian(in1, in2, in3);