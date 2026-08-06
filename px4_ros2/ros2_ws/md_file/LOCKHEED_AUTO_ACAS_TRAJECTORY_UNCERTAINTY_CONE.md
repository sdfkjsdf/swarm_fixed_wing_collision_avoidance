# Lockheed Martin Auto ACAS trajectory uncertainty cone

> 상태: 논문 분석 및 향후 구현 방향 기록
>
> 범위: Lockheed Martin Auto ACAS 논문에서 설명하는 trajectory cone,
> 궤적 불확실성, PMR/MASD/AD 관계를 현재 프로젝트 관점에서 정리한다.
> 이 문서의 "현재 구현 상태"와 "구현 제안"은 논문 원문과 구분한다.

## 1. 결론

Auto ACAS는 하나의 예측 궤적 선만 충돌 검사에 사용하지 않는다. 예측 궤적
주변에 시간에 따라 증가하는 불확실성 영역을 두며, 논문은 이 영역을
`trajectory cone`이라고 부른다.

두 항공기의 cone은 각각 다음을 나타낸다.

- 중심선: TPA가 계산한 nominal predicted trajectory
- cone 폭: 해당 시각에 항공기가 실제로 존재할 수 있는 공간 범위
- 시간 방향: 현재 시각부터 예측 horizon 끝까지

따라서 그림의 cone은 단순한 3차원 원뿔 한 개라기보다, 곡선 궤적을 따라
단면이 커지는 3차원 uncertainty tube를 시간축까지 포함해 표현한 개념이다.
2012 논문은 이를 시간과 공간의 4차원 문제라고 설명한다.

두 cone이 같은 미래 시각과 공간에서 접촉하면 회피 기동 활성화가 필요하다고
판단한다. 2013 논문에서는 이 조건을 `AD = PMR - MASD`로 설명하고,
`AD < 0`이면 회피 기동을 활성화한다.

## 2. 논문 근거

### 2.1 2012 algorithm design paper

`4.6 Maneuver Activation and Control`, PDF 8쪽의 Figure 4
`Trajectory Cones`에서 다음을 설명한다.

- 각 항공기의 best avoidance trajectory 주변에 불확실성을 포함한다.
- 이 영역은 항공기가 비행할 가능성이 있는 공간의 체적을 나타낸다.
- 충돌 위험이 증가하면 두 cone이 시간과 공간에서 수렴한다.
- 두 cone이 접촉하는 순간 선택된 회피 기동을 요청한다.
- 최종 safety check는 잘못된 기동 조합을 금지하기 위해 cone의 시공간
  penetration도 검사한다.

논문 경로:

- [Automatic Aircraft Collision Avoidance Algorithm Design for Fighter Aircraft](../../../../../../reference/paper/Lockheed_Martin_collision_avoidance/%5B95%5DAutomatic%20Aircraft%20Collision%20Avoidance%20Algorithm%20Design%20for%20Fighter%20Aircraft.pdf)

### 2.2 2013 development paper

`C. Ownship Aircraft Trajectory Prediction`, PDF 7쪽에서는 다음 수치와 목표를
제시한다.

- TPA prediction horizon: 4.5 s
- 외부 모듈로 제공하는 궤적: 0.1 s 간격의 46개 점
- 정확도 목표 예시: 궤적 3 s 지점에서 high-fidelity trajectory와 60 ft 이내
- TPA uncertainty model의 coverage 목표: off-line 6DOF 비교에서 관측된 실제
  오차의 95%가 uncertainty model 출력 범위 안에 포함되도록 설정

여기서 95%는 이 논문이 구체적인 확률분포나 covariance 전파식을 제공한다는
뜻이 아니다. high-fidelity 6DOF truth와 TPA 결과의 경험적 오차를 이용해
uncertainty envelope가 실제 오차의 95%를 덮도록 튜닝한다는 coverage 목표다.

`E. Maneuver Activation and Control`, PDF 9쪽의 Figure 8
`AMAC Activation Computation`에서는 cone 폭에 포함되는 불확실성 원천과
활성화 조건을 설명한다.

논문 경로:

- [Development of an Automatic Aircraft Collision Avoidance](../../../../../../reference/paper/Lockheed_Martin_collision_avoidance/Development%20of%20an%20Automatic%20Aircraft%20Collision%20Avoidance.pdf)

## 3. PMR, MASD, AD 관계

논문의 활성화 판단은 다음 세 양으로 정리할 수 있다.

### 3.1 PMR - Predicted Minimum Range

TPA가 생성한 ownship과 threat의 nominal trajectory 사이에서 예측되는 최소
거리다. 구현 관점에서 다음과 같이 해석할 수 있다.

```text
PMR = min over t of distance(p_own(t), p_threat(t))
```

위 식은 논문의 개념을 코드 관점으로 표기한 것이다. 논문은 이 문서에서
구체적인 minimizer 구현이나 보간 알고리즘까지 공개하지 않는다.

### 3.2 MASD - Minimum Allowed Separation Distance

2013 논문에 따르면 MASD는 다음 항목을 합친 최소 허용 분리 거리다.

```text
MASD = ownship half-wingspan
     + threat half-wingspan
     + DSD
     + rolled-up uncertainties
```

`DSD(Desired Separation Distance)`는 조종사가 입력할 수 있는 고정 분리
거리다. 불확실성은 미래로 갈수록 증가할 수 있으므로 실제 구현에서는
MASD를 `MASD(t)`로 취급하는 편이 자연스럽다.

### 3.3 AD - Avoidance Distance

```text
AD = PMR - MASD
```

- `AD > 0`: 예측 최소 거리 안에 아직 안전 여유가 있음
- `AD = 0`: 두 uncertainty cone이 접촉하는 경계
- `AD < 0`: 허용 분리 영역이 겹치므로 회피 기동 활성화 필요

개념적으로는 다음과 같다.

```text
ownship nominal path       threat nominal path
        \                         /
         \==== uncertainty ======/
          \===== envelope ======/
                 ^
                 |
          cone contact: AD = 0
```

## 4. 논문이 열거한 uncertainty 원천

2013 논문의 MASD uncertainty roll-up에는 다음 일곱 종류가 포함된다.

1. Navigation uncertainty
   - horizontal navigation error
   - vertical navigation error
2. Trajectory generation uncertainty
   - 저차 TPA와 실제 항공기 응답의 차이
3. Trajectory reconstruction/splining uncertainty
   - 압축된 trajectory sample을 spline으로 복원하면서 생기는 오차
4. Datalink resolution uncertainty
   - 전송 자료형의 least significant bit 및 양자화 오차
5. Trajectory minimization uncertainty
   - 이산 시각 또는 수치 최적화로 minimum range를 찾을 때 생기는 오차
6. Pilot input and transmission delay uncertainty
   - 조종 입력 변화와 송수신 지연 때문에 생기는 시간 정렬 오차
7. Coupler modeling uncertainty
   - TPA/coupler 모델과 high-fidelity 6DOF 응답의 차이

중요한 점은 cone이 TPA 모델 오차 하나만 나타내는 것이 아니라는 것이다.
항법, 예측, spline 복원, 통신, 최소거리 탐색, 지연, 실제 비행제어 응답 오차를
합친 최종 safety envelope다.

## 5. 논문에서 공개하지 않은 부분

두 논문은 cone 개념과 uncertainty 항목은 공개하지만 다음 세부 식은 제공하지
않는다.

- cone 단면이 원, 타원 또는 다른 형상인지에 대한 완전한 수학적 정의
- 각 uncertainty 항목의 시간별 수치 또는 growth-rate table
- 여러 uncertainty를 scalar radius로 합치는 정확한 roll-up 공식
- 항목 사이 상관관계와 covariance 처리 방식
- horizontal/vertical uncertainty의 정확한 confidence mapping

따라서 Figure 4와 Figure 8의 모양만 보고 고정 각도의 right circular cone이나
Gaussian covariance ellipsoid라고 단정하면 안 된다. 현재 확인 가능한 가장
정확한 표현은 "예측 궤적을 따라 시간이 지나며 성장하는 uncertainty
envelope"다.

## 6. 현재 프로젝트 구현과의 대응

### 6.1 구현된 부분

현재 프로젝트에는 cone의 중심선을 만들고 압축/복원하는 기반이 있다.

```text
PredictState + PredictInput
        |
        v
TrajectoryPredict: 0.0 - 4.5 s, 46 points
        |
        v
TrajectorySample: positions at 0.0/1.5/3.0/4.5 s
                  velocities at 0.0/4.5 s
        |
        v
ReconstructTrajectory: clamped cubic spline
```

- `TrajectoryPredict`: nominal trajectory 생성
- `TrajectorySample`: 데이터링크용 핵심 sample 표현
- `ReconstructTrajectory`: 수신 측 nominal trajectory 복원
- `trajectory_prediction_hils`: nominal prediction과 PX4 SITL 측정 궤적 비교
- `analysis_tools`: 시간별 위치/속도/자세 오차 분석

### 6.2 아직 구현되지 않은 부분

현재 코드에는 다음 기능이 없다.

- 시간별 trajectory uncertainty 자료형
- TPA residual의 95% envelope 산출 및 lookup table
- navigation/spline/datalink/delay uncertainty roll-up
- ownship/threat cone 생성
- PMR, MASD, AD 계산
- cone contact/overlap 판단
- AD 기반 maneuver activation

따라서 현재 구현은 Auto ACAS 흐름 중 nominal trajectory prediction과
trajectory reconstruction까지 구현된 상태이며, uncertainty cone과 AMAC 판단은
후속 collision-estimation 단계다.

## 7. 현재 HILS 데이터를 이용한 구현 방향

다음 방식이면 기존 시험 환경을 그대로 이용해 첫 번째 uncertainty model을
만들 수 있다.

### Phase 1 - TPA empirical residual 수집

예측을 시작한 시각을 `t0`, horizon index를 `k`라고 할 때 다음 residual을
계산한다.

```text
e_n(k) = p_measured_n(t0 + k*dt) - p_predicted_n(k)
e_e(k) = p_measured_e(t0 + k*dt) - p_predicted_e(k)
e_h(k) = p_measured_h(t0 + k*dt) - p_predicted_h(k)

e_horizontal(k) = sqrt(e_n(k)^2 + e_e(k)^2)
e_vertical(k)   = abs(e_h(k))
```

속도, 고도, roll, 상승/하강, 입력 변화율별 HILS case를 반복 실행해 각
horizon 시각의 residual 분포를 모은다.

### Phase 2 - 95% TPA envelope 생성

각 horizon 시각에서 residual의 경험적 95 percentile을 구한다.

```text
u_tpa_horizontal(k) = percentile95(e_horizontal(k))
u_tpa_vertical(k)   = percentile95(e_vertical(k))
```

샘플 부족으로 envelope가 불규칙하게 줄어들지 않도록 보수적인 smoothing과
monotonic growth 제약을 검토한다. 단, 항상 단조 증가해야 한다는 조건은 논문
명세가 아니라 초기 안전 구현을 위한 프로젝트 선택이다.

### Phase 3 - spline uncertainty 분리

현재 `compare_spline_reconstruct.py`를 이용해 원본 46점 trajectory와 복원된
spline 사이의 오차를 시각별로 측정한다.

```text
u_spline_horizontal(k)
u_spline_vertical(k)
```

TPA-vs-SITL 오차와 spline-vs-TPA 오차를 분리해야 이중 계상을 피할 수 있다.

### Phase 4 - total envelope와 separation 판단

초기 보수 모델에서는 uncertainty 항목을 모두 거리 bound로 변환한 뒤 합산할
수 있다.

```text
u_total(t) = u_navigation(t)
           + u_tpa(t)
           + u_spline(t)
           + u_datalink(t)
           + u_minimization(t)
           + u_delay(t)
           + u_coupler(t)
```

그 후 ownship과 threat의 envelope를 함께 고려한다.

```text
separation_margin(t)
    = distance(p_own(t), p_threat(t))
    - physical_size_margin
    - DSD
    - u_total_own(t)
    - u_total_threat(t)

AD = min over t of separation_margin(t)
```

이 합산식은 논문의 공개된 구현식이 아니라, 각 항목을 independent probability로
간주하지 않고 worst-case distance bound로 다루는 초기 프로젝트 제안이다.
통계적 covariance 또는 ellipsoid 모델을 도입할 경우 별도의 검증 근거가
필요하다.

## 8. 권장 모듈 경계

향후 구현 시 production과 HILS를 다음처럼 분리하는 구성이 적절하다.

```text
collision_avoidance/
├── include/collision_avoidance/estimation/
│   ├── trajectory_prediction/
│   ├── reconstruction/
│   └── uncertainty/
│       ├── UncertaintyTypes.hpp
│       └── TrajectoryUncertaintyModel.hpp
└── src/estimation/uncertainty/
    └── TrajectoryUncertaintyModel.cpp

testing_module/
├── trajectory_prediction_hils/
└── analysis_tools/
    └── estimate_uncertainty_envelope.py
```

권장 자료형의 최소 개념은 다음과 같다.

```cpp
struct TrajectoryUncertaintyPoint {
    float horizontal_radius_m;
    float vertical_radius_m;
};

struct SeparationResult {
    float predicted_minimum_range_m;
    float minimum_allowed_separation_m;
    float avoidance_distance_m;
    float time_of_closest_approach_s;
};
```

이 자료형과 수치는 아직 구현 명세가 아니라 후속 설계를 위한 출발점이다.

## 9. 구현 전 결정해야 할 사항

1. 첫 모델을 scalar sphere, horizontal/vertical cylinder, ellipsoid 중 어떤
   형상으로 표현할 것인가?
2. 95% coverage를 전체 비행영역 하나로 보장할 것인가, flight-condition bin별로
   보장할 것인가?
3. PX4 EKF가 제공하는 position uncertainty를 직접 사용할 수 있는가?
4. 통신 지연을 위치 오차 `speed * delay`로 변환할 것인가?
5. ownship과 cooperating threat가 uncertainty metadata까지 전송할 것인가?
6. non-cooperating threat에는 어떤 별도 prediction uncertainty를 적용할 것인가?
7. cone 접촉을 0.1 s 이산 점에서만 검사할 것인가, spline 구간 안의 연속 최소
   거리를 계산할 것인가?

## 10. 핵심 요약

```text
nominal trajectory만 비교하는 것
    != Lockheed Martin Auto ACAS의 최종 충돌 판단

nominal trajectory
    + 시간에 따라 성장하는 uncertainty envelope
    + 기체 크기
    + 원하는 분리 거리
    = trajectory cone / MASD 판단 기반
```

현재 프로젝트는 cone의 중심선 생성, 압축, 전송 prototype, spline 복원까지
준비되어 있다. 다음 단계는 HILS residual로 TPA와 spline uncertainty를 정량화한
뒤, 나머지 오차원을 보수적으로 roll-up하여 PMR/MASD/AD 기반 collision
estimation으로 연결하는 것이다.
