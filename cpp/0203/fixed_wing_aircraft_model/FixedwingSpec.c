/******************************************************** */
/* 작성자 : 이동혁    */
/* 작성날짜 : 2025/11/22일 */
/* 기능 세부 설명 : 고정익 비행기의 최대 최고 속력, 최대 허용 롤 피치각을 정의하는 함수 fixed_wing_spec.h의 세부 구현 내용을 의미 */ 

/*  수정날짜 및 수정사항    */ 
/* 현재 비행기 동역학이 바뀐 관계로 전반적인 상태 차원  및 수정에 들어감 2025/12/31 */

/********************************************************* */


#define _USE_MATH_DEFINES /* 지금 pi을 사용하기 위해서 선언 */
#include "FixedwingSpec.h"
#include <math.h>

#define DEG_TO_RAD(deg)   ((deg) * (M_PI / 180.0))  /* 매크로 함수 지금 도를 라디안으로 변환시키기 위해서 사용 */

const FixedwingSpec_t g_FixedwingSpec = {

     .max_speed = 45.0, /* 단위  m/s */
     .min_speed = 15.0,  /* 단위  m/s */
     .max_roll  = DEG_TO_RAD(45.0),  /* 최대 허용 롤각 선언 */ 
     .min_roll  = -DEG_TO_RAD(45.0),  /* 최소 허용 롤각 선언 */ 
     .max_pitch = DEG_TO_RAD(20.0), /* 최대 허용 피치각 선언 */ 
     .min_pitch = -DEG_TO_RAD(15.0), /* 최소 허용 피치각 선언 */ 
     .timeconstant_p = 0.07,
     .timeconstant_q = 0.07,
     .timeconstant_r = 0.07,
     .timeconstant_at =0.05,
     .p_max =  DEG_TO_RAD(90),      /* 최대 roll rate */ 
     .p_min = -DEG_TO_RAD(90),      /* 최소 roll rate */ 
     .q_max =  DEG_TO_RAD(45),      /* 최대 pitch rate*/ 
     .q_min = -DEG_TO_RAD(45),      /* 최소 pitch rate */ 
     .r_max =  DEG_TO_RAD(30),      /* 최대 yaw rate */ 
     .r_min = -DEG_TO_RAD(30),      /* 최소 yaw rate */ 
     .at_max = 3.5,                   /* 최대 T/m 즉 중량 대 추력비  */ 
     .at_min = 0.0,     
     .g = 9.81,                     /*중력 가속도를 의미 */
     .k_acc = 0.02/(2.0*20)   /*항력 계수를 의미*/


};