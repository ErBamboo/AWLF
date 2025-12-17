
#include "core/awlf_def.h"
#include "data_struct/corelist.h"

#define MOTOR_NAME_MAX_LENGTH 16
typedef struct EcdMotor* EcdMotor_t;

typedef struct EcdMotorParam
{
    float angle;       // 单位：弧度
    float speed;       // 单位：rad/s，采用弧度制，便于微积分运算
    float torque;      // 单位：Nm, 缺省值为0.0f
    float current;     // 单位：A，缺省值为0.0f
    float tempurature; // 单位：摄氏度
} EcdMotorParam_s;

typedef struct EcdMotorSetting
{
    float    reductionRatio; // 电机减速比，输出转速 = 输入转速 / 减速比
    uint32_t feedbackId;     // 电机反馈报文ID
    uint32_t commandId;      // 电机命令报文ID

    // 实验功能，待开发
    float zeroAngle;       // 单位：弧度，电机零点位置，电机使能后自动运行到该位置
    float angleLimitUpper; // 单位：弧度，绝对角度上限，超出范围会被截断
    float angleLimitLower; // 单位：弧度，绝对角度下限，超出范围会被截断
    float maxSpeed_abs;    // 单位：rad/s，绝对值最大速度
    float maxTorque;       // 单位：Nm，绝对值最大扭矩
    float maxTempurature;  // 单位：摄氏度，最大温度限制，超出范围会警告
} EcdMotorSetting_s;

typedef struct EcdMotorInterface* EcdMotorInterface_t;
typedef struct EcdMotorInterface
{
    /**
     * @brief 控制电机，根据期望参数更新实际参数
     *
     * @param motor 电机指针
     * @param expParam 期望参数
     * @return AwlfRet_e 状态码
     */
    AwlfRet_e (*control)(EcdMotor_t motor, EcdMotorParam_s expParam);

    /**
     * @brief 更新电机参数，根据反馈报文更新实际参数
     *
     * @param motor 电机指针
     * @return EcdMotorParam_s 实际参数
     */
    EcdMotorParam_s (*update)(EcdMotor_t motor);

} EcdMotorInterface_s;

typedef struct EcdMotorState
{
    EcdMotorParam_s currentParam; // 实际参数，实时记录，调试跟踪时可用
    EcdMotorParam_s expParam;     // 期望参数，每个控制环路实时更新，记录过程量，调试跟踪时可用
    // TODO: 后台守护模块，记录电机状态，并打印日志
} EcdMotorState_s;

typedef struct EcdMotor
{
    char                name[MOTOR_NAME_MAX_LENGTH]; // 电机名称
    EcdMotorSetting_s   setting;                     // 电机设置
    EcdMotorState_s     state;                       // 电机状态
    EcdMotorInterface_t interface;                   // 电机驱动接口
} EcdMotor_s;

AwlfRet_e motor_register(const char* name, size_t nameLen, EcdMotor_t motor, EcdMotorInterface_t interface);

static inline EcdMotorParam_s motor_get_current_param(EcdMotor_t motor)
{
    return motor->state.currentParam;
}

static inline float motor_get_angle(EcdMotor_t motor)
{
    return motor->state.currentParam.angle;
}

static inline float motor_get_speed(EcdMotor_t motor)
{
    return motor->state.currentParam.speed;
}

static inline float motor_get_torque(EcdMotor_t motor)
{
    return motor->state.currentParam.torque;
}

static inline float motor_get_current(EcdMotor_t motor)
{
    return motor->state.currentParam.current;
}

static inline float motor_get_tempurature(EcdMotor_t motor)
{
    return motor->state.currentParam.tempurature;
}
