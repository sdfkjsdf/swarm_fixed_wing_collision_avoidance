   
   
<  지금 system id 하고  토픽 prefix 하고 마블로스 링크 때문에 어쩔 수 없이 1부터 시작이 되는 것을 써아함>

   
   ────────────┬─────────────┬───────────┐                                                                                                                                                                         
  │ vehicle_ID │ 토픽 prefix │ system_id │                                                                                                                                                                         
  ├────────────┼─────────────┼───────────┤                                                                                                                                                                         
  │ 1          │ /px4_1/     │ 2         │
  ├────────────┼─────────────┼───────────┤                                                                                                                                                                         
  │ 2          │ /px4_2/     │ 3         │
  ├────────────┼─────────────┼───────────┤
  │ 3          │ /px4_3/     │ 4         │
  ├────────────┼─────────────┼───────────┤                                                                                                                                                                         
  │ 4          │ /px4_4/     │ 5         │
  ├────────────┼─────────────┼───────────┤                                                                                                                                                                         
  │ 5          │ /px4_5/     │ 6         │
  └────────────┴─────────────┴───────────┘

  - 토픽: /px4_ + m_vehicle_ID (그대로)                                                                                                                                                                            
  - system_id: m_vehicle_ID + 1 (이미 수정한 것)
                                                                                                                                                                                                                   
  선형대수 인덱싱할 때만 vehicle_ID - 1로 0-based 변환하면 됩니다.  


  <비행기의 초기조건을 설정하는 방법>

   Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s standard_vtol:1:0:0,standard_vtol:1:100:0,standard_vtol:1:200:0,standard_vtol:1:0:100,standard_vtol:1:100:100                                            
                                                                                                                                                                                                                   
    형식: 모델:대수:X:Y     

   Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s standard_vtol:1:0:0,standard_vtol:1:0:100,standard_vtol:1:0:200,standard_vtol:1:0:300,standard_vtol:1:0:400 
   Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s standard_vtol:1:0:0,standard_vtol:1:0:50,standard_vtol:1:0:100,standard_vtol:1:0:150,standard_vtol:1:0:200 
   Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s standard_vtol:1:0:0,standard_vtol:1:50:0,standard_vtol:1:95:0,standard_vtol:1:150:0,standard_vtol:1:200:0 



   Gazebo는 ENU 좌표계를 씁니다:                                                                                                                                                                                    
                                                                                                                                                                                                                   
  - X (빨간색) = East (동쪽)                                                                                                                                                                                       
  - Y (초록색) = North (북쪽)                                                                                                                                                                                      
  - Z (파란색) = Up                                                                                                                                                                                                
                                                                                                                                                                                                                   
  PX4는 NED를 씁니다:                                                                                                                                                                                              
  - X = North                                                                                                                                                                                                      
  - Y = East      
            
  즉 Gazebo와 PX4의 X, Y가 서로 뒤바뀌어 있습니다.  