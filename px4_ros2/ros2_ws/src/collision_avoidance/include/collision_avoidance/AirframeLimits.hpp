#pragma once

/*
   AirframeLimits
   --------------
   고정익 에너지 성능 기반 dV/dt 클램프.

   ArduPilot / PX4 TECS 방식:
     dv_max =  energy_fraction * max_climb_rate * g / TAS
     dv_min = -energy_fraction * min_sink_rate  * g / TAS

   모든 값은 airframe_spec.yaml 에서 로드하여 외부에서 주입.
   FlockingGuidance, CollisionAvoidance 등 가이던스 모듈에서 공용 사용.
*/

#include <algorithm>
#include <cmath>

struct AirframeLimits
{
    float max_climb_rate;       /* FW_T_CLMB_MAX [m/s] — yaml: height_rate_max_climb */
    float min_sink_rate;        /* FW_T_SINK_MIN [m/s] — yaml: height_rate_min_sink  */
    float airspeed_min;         /* stall 방지 분모 가드  — yaml: airspeed_min          */
    float energy_fraction;      /* 속도 배분 비율        — yaml: energy_fraction        */
    float gravity;              /* 중력가속도 [m/s^2]    — yaml: gravity                */

    float dvdtMax(float tas) const
    {
        return energy_fraction * max_climb_rate * gravity
               / std::max(tas, airspeed_min);
    }

    float dvdtMin(float tas) const
    {
        return -energy_fraction * min_sink_rate * gravity
               / std::max(tas, airspeed_min);
    }

    float clampDvDt(float dv_dt, float tas) const
    {
        return std::clamp(dv_dt, dvdtMin(tas), dvdtMax(tas));
    }
};
