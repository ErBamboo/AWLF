#ifndef __GM6020_H__
#define __GM6020_H__

#include "algorithm/controller/pid.h"
#include "component/driver/pal/pal_can_dev.h"
#include <math.h>

#define GM6020_SEND_ID       (0x1FFU) // 发送标识符，当前表示电压发送
#define GM6020_ID            (1U)     // GM6020电机ID
#define GM6020_FEEDBACK_BASE (0x204U)
#define GM6020_FEEDBACK_ID   (GM6020_FEEDBACK_BASE + GM6020_ID) // 反馈报文ID

#ifndef PI
    #define PI (3.1415f)
#endif

#define RPM_2_RADPS(rpm)      ((2.0f * (rpm) * PI) / 60.0f)                                                      // rpm ——> rad/s
#define RADPS_2_RPM(radps)    ((60.0f * (radps)) / (2.0f * PI))                                                  // rad/s ——> rpm
#define RPM_2_ECD(motor, rpm) (int16_t)(((rpm) / (motor)->constVelPerVolt) * ((motor)->maxVoltageValue / 24.0f)) // rpm ——> 编码值

#define ANGLE_2_RAD(angle)    ((angle) * (PI / 180.0f)) // 角度 ——> rad

#define ECD_2_ANGLE(ecd)      ((ecd) * (360.0f / 8192.0f))  // 编码值 ——> 角度
#define ECD_2_RAD(ecd)        ANGLE_2_RAD(ECD_2_ANGLE(ecd)) // 编码值 ——> rad

typedef enum
{
    GM6020_STATUS_SHUTDOWN = 0, // 关闭状态
    GM6020_STATUS_INIT,         // 初始化状态
    GM6020_STATUS_RUNNING,      // 运行状态
} GM6020Status_e;

typedef struct GM6020* GM6020_t;
typedef struct GM6020
{
    Device_t        canHandler;
    uint8_t         settingCurrunt[8];
    uint8_t         feedbackValue[8]; // 反馈值
    PidController_s speedPid;
    PidController_s anglePid;
    GM6020Status_e  status;
    uint8_t         initSampleCnt; // 初始化采样次数

    float           speedFb;        // 速度前馈
    float           speedFbCoef;    // 速度前馈系数
    float           accFb;          // 加速度前馈
    float           accFbCoef;      // 加速度前馈系数

    struct
    {
        uint8_t  motorID;
        uint32_t sendID;
        uint32_t FeedbackID;
        float    constVelPerVolt; // 速度与电压的比例关系
        uint32_t maxVoltageValue; // 最大电压值 abs(25000)
    } settings;

    struct
    {
        float   velocity;    // 速率，单位 rpm
        int32_t totalRound;  // 总圈数（有向）
        float   totalAngle;  // 总角度（有向）, 单位度
        float   singleAngle; // 单圈角度, 单位度
        int16_t currunt;     // 电流值
        uint8_t temperature; // 摄氏度
        float   filterSpeed; // 滤波后的编码器速度, 单位rpm
        int16_t offsetEcd;   // 角度偏移值, 单位度
    } statusValue_s;         // 状态量
    struct
    {
        int16_t angleEcd;     // 机械角度 0-8191
        int16_t speed;        // 单位rpm
        int16_t lastAngleEcd; // 上一周期的编码器角度, 单位机械角度 0-8192
    } rawData;                // 原始数据
    CanUserMsg_s feedbackMsg;
    CanUserMsg_s sendMsg;
} GM6020_s;

void gm6020_set_acc_feedback(GM6020_t gm6020, float accFb, float accFbCoef);
void gm6020_set_speed_feedback(GM6020_t gm6020, float speedFb, float speedFbCoef);
void gm6020_update(GM6020_t gm6020);
void gm6020_set_speed(GM6020_t gm6020, float speed_rpm);
void gm6020_set_single_angle(GM6020_t gm6020, float setAngleDeg);
void gm6020_init(GM6020_t gm6020, Device_t canHandler);

#endif
