#ifndef LEADERFOLLOWERPARAMETER_H
#define LEADERFOLLOWERPARAMETER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 

{
      double m_k_v; 
      double m_k_dy;
      double m_k_dot_dy;
      double m_k_dx;
      double m_lookahead_distance;
      double m_k_smooth;
      








    /* data */
}LeaderFollowerParameter_t;





extern const LeaderFollowerParameter_t g_LeaderFollowerParameter; /* g_fixed_wing_spec 은  기체 모두 동일한 스펙을 가정함으로  global 변수로 선언 */


#ifdef __cplusplus
}
#endif









#endif