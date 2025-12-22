#ifndef __MOTOR__H__
#define __MOTOR__H__

#include "aw_core/awlf_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO: 加入离线检测模块

typedef struct
{
    float gearRatio;         // 减速比

    struct
    {
        float maxVelocityRadps;  // 最大速度, 单位: 弧度/秒
        float maxTorqueNm;       // 最大扭矩, 单位: 牛米
        float maxTotalAngleRad;  // 最大累计角度, 单位: 弧度
        
        float minTotalAngleRad;  // 最小累计角度, 单位: 弧度
        float minVelocityRadps;  // 最小速度, 单位: 弧度/秒
        float minTorqueNm;       // 最小扭矩, 单位: 牛米
    } limit;
} MotorSettings_s;

typedef struct MotorDevice
{
    float velocityRadps; // 电机速度, 单位: 弧度/秒
    float angleRad;      // 电机角度, 单位: 弧度
    float currentMA;     // 电机电流, 单位: 毫安
    float torqueNm;      // 电机扭矩, 单位: 牛米
    
} MotorDevice_s;



#ifdef __cplusplus
}
#endif

#endif
