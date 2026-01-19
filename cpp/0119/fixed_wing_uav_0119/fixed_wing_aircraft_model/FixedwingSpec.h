
/******************************************************** */
/* 작성자 : 이동혁    */
/* 작성날짜 : 2025/11/22일 */
/* 기능 세부 설명 : 고정익 비행기의 최대 최고 속력, 최대 허용 롤 피치각 및 동역학적 지연모델을 정의  */ 

/*  수정날짜 및 수정사항    */ 
/* 현재 비행기 동역학이 바뀐 관계로 전반적인 상태 차원  및 수정에 들어감 2025/12/31 */


/********************************************************* */

#ifndef FIXED_WING_SPEC_H
#define FIXED_WING_SPEC_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct

{
    /*비행기 spec 적인 부분의 정의*/

        double max_speed ; /* 최고 속력를 정의 */ 
        double min_speed ; /* 최저 속력을 정의 */ 
        double max_roll  ; /* 최대 허욜  롤각 선언 */ 
        double min_roll  ; /* 최소 허용 롤각 선언 */ 
        double max_pitch ; /* 최대 허용 피치각 선언 */ 
        double min_pitch ; /* 최소 허용 피치각 선언 */ 


        /* 동역학적 시간 지연 모델을 설정 */
        double timeconstant_p ;
        double timeconstant_q ;
        double timeconstant_r ;
        double timeconstant_at;

       /*Control input boundary을 설정 */
        double p_max ;      /* 최대 roll rate */ 
        double p_min ;      /* 최소 roll rate */ 
        double q_max  ;     /* 최대 pitch rate*/ 
        double q_min  ;     /* 최소 pitch rate */ 
        double r_max ;      /* 최대 yaw rate */ 
        double r_min ;      /* 최소 yaw rate */ 
        double at_max ;     /* 최대 T/m 즉 중량 대 추력비  */ 
        double at_min ;     /* 최소 T/m 즉 중량 대 추력비  */ 

        double g ;
        double k_acc;





}FixedwingSpec_t; /* 구조체 타입 이름 선언 */  

extern const FixedwingSpec_t g_FixedwingSpec; /* g_fixed_wing_spec 은  기체 모두 동일한 스펙을 가정함으로  global 변수로 선언 */


#ifdef __cplusplus
}
#endif


#endif /* FIXED_WING_SPEC_H */ 

///////////////////////////////////////////////////////

