#ifndef APFPARAMETER_H
#define APFPARAMETER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 

{
      double m_alpha;
      double m_a;
      double m_b; 
     

    /* data */
}APFParameter_t;



extern const APFParameter_t g_APFParameter; /* g_fixed_wing_spec 은  기체 모두 동일한 스펙을 가정함으로  global 변수로 선언 */


#ifdef __cplusplus
}
#endif






#endif