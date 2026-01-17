#ifndef FLOCKINGPARAPMETER_H
#define FLOCKINGPARAPMETER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 

{
      double m_beta;
      double m_lammda;
      double m_k1;
      double m_k2;
      double m_desired_distance;








    /* data */
}Flockingparameter_t;





extern const Flockingparameter_t g_Flockingparameter; /* g_fixed_wing_spec 은  기체 모두 동일한 스펙을 가정함으로  global 변수로 선언 */


#ifdef __cplusplus
}
#endif









#endif 