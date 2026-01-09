
/***************************************************************/
/*  작성자 : 이동혁   */

/** 작성날짜 : 2026/01/01일  **/
/* 현재 롤 피치 요 setpoint 생성을 위한 게인 파라미터 선언*/





/**************************************************************/

#ifndef GAINPARAMETER_H
#define GAINPARAMETER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct

{
    /*roll pitch yaw setpoint 생성을 위한  gain 파리미터 부분*/

        double k_yaw ;      /* 해당 gain은 yaw_error을 통해서 d(yaw_error)/dt을 구함 */ 
        double k_roll ;     /* 해당 gain 은 dot_roll_sp=k_roll*(roll_sp-roll); 을 만들 때 사용 */ 
        double k_d  ;       /* 해당 gain은 error_d=d_set-d; dot_error_d=k_d*(error_d); 을 만들 떄 사용 */ 
        double k_pitch  ;   /* 해당 gain은  dot_pitch_sp=k_pitch*(pitch_sp-pitch) */ 
        double k_v ;        /* 해당 gain은 error_v=v_sp_sat-v;  dot_error_v=k_v*(error_v); 으로 추력 채널에서 사용되는 부분임  */
      

}GainParameter_t; /* 구조체 타입 이름 선언 */  

extern const GainParameter_t g_GainParameter; /* g_fixed_wing_spec 은  기체 모두 동일한 스펙을 가정함으로  global 변수로 선언 */


#ifdef __cplusplus
}
#endif


#endif /* FIXED_WING_SPEC_H */ 

///////////////////////////////////////////////////////

