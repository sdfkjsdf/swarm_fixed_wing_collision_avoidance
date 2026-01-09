
/***************************************************************/
/*  작성자 : 이동혁   */

/** 작성날짜 : 2026/01/01일  **/
/* 현재 NDI 에서 사용하기 위한 파라미터를 선언하는 내용 */





/**************************************************************/

#ifndef NDIPARAMETER_H
#define NDIPARAMETER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct

{
    /*roll pitch yaw setpoint 생성을 위한  gain 파리미터 부분*/

        double k_p_sp ;      
        double k_q_sp ;     
        double k_r_sp  ;      
        double k_at_sp  ;   
        
      

}NdiParameter_t; /* 구조체 타입 이름 선언 */  

extern const NdiParameter_t g_NdiParameter; /* g_fixed_wing_spec 은  기체 모두 동일한 스펙을 가정함으로  global 변수로 선언 */


#ifdef __cplusplus
}
#endif


#endif /* FIXED_WING_SPEC_H */ 

///////////////////////////////////////////////////////

