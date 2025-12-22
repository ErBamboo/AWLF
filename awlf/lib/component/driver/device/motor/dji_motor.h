#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "data_struct/corelist.h"
#include "motor.h"
#include "component/driver/pal/pal_can_dev.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DJI_MOTOR_MAX_NUM_PER_BUS     (8U) // 一个CAN总线上最多支持8个大疆电机

/**
 * @brief DJI_ESC_CTRL_MODE
 *
 */
#define DJI_MOTOR_CTRL_VOLTAGE (0x01U) // 电压控制
#define DJI_MOTOR_CTRL_CURRENT (0x02U) // 电流控制

/**
 * @brief DJI_ESC_TYPE
 *
 */
#define DJI_ESC_C610   (0x01U) // C610电调
#define DJI_ESC_C620   (0x02U) // C620电调
#define DJI_ESC_GM6020 (0x03U) // GM6020电调

typedef struct DJIMotorBus* DJIMotorBus_t;
typedef struct DJIMotorGroup* DJIMotorGroup_t;
typedef struct DJIMotor* DJIMotor_t;
typedef struct DJIMotorESC* DJIMotorESC_t;

/**
 * @brief 大疆电调规定的错误码，按手册规定顺序排列
 * @note GM6020是没有错误码反馈的，因此部分错误码无法获取
 */
typedef enum
{
    DJI_MOTOR_ERR_NONE = 0,                       // 无错误
    DJI_MOTOR_ERR_FAIL_TO_ACCESS_MEM_CHIP,        // 访问内存芯片失败
    DJI_MOTOR_ERR_OVER_VOLTAGE,                   // 电压过高
    DJI_MOTOR_ERR_THREE_PHASE_LINE_NOT_CONNECTED, // 三相线未接入
    DJI_MOTOR_ERR_LOSS_OF_POSITION_SENSOR_DATA,   // 丢失位置传感器数据
    DJI_MOTOR_ERR_MOTOR_STALL,                    // 电机堵转
    DJI_MOTOR_ERR_EXCEPTION_OR_TEMPERATURE_HIGH,  // 电机异常或温度过高 (温度 > 180℃)
    DJI_MOTOR_ERR_CALIBRATION_FAILED,             // 电机校准失败
    DJI_MOTOR_ERR_OVER_TEMPERATURE,               // 电机过热
} DJIMotorErrCode_e;

/**
 * @brief 大疆电机电调
 */
typedef struct DJIMotorESC
{
    uint8_t type;     // 电调类型, @ref DJI_ESC_TYPE
    uint8_t ctrlMode; // 控制模式, @ref DJI_ESC_CTRL_MODE
    uint8_t escId;    // 电调ID，对应DJI电调的实际ID

    uint32_t feedbackBasedId; // 手册中电调反馈标识符的基数，比如GM6020的反馈标识符基数是0x204，C610和C620是0x200
    uint32_t sendId;          // DJI手册对应的电调接收标识符（电调是接收方，那MCU就是发送方，所以取名sendId）
} DJIMotorESC_s;

typedef struct DJIMotorCfg* DJIMotorCfg_t;
typedef struct DJIMotorCfg
{
    uint8_t escType;  // 电调类型, @ref DJI_ESC_TYPE
    uint8_t ctrlMode; // 控制模式, 一些电调只支持电流控制 @ref DJI_ESC_CTRL_MODE
    uint8_t escId;    // 电调ID，对应DJI电调的实际ID
} DJIMotorCfg_s;

typedef struct DJIMotor
{
    DJIMotorGroup_t ownerBus; // 所属的电机总线
    uint32_t groupIdx;         // 电机在所属group中的索引
    DJIMotorESC_s esc;        // 电调
    int8_t errCode;           // 错误码，缺省为-1
    struct
    {
        int16_t angleEcd;          // 机械角度 0-8191
        int16_t velocity;          // 单位rpm
        int16_t current;           // 电流，单位毫安
        int16_t temperature;       // 温度，单位摄氏度
        DJIMotorErrCode_e errcode; // 错误码
        int16_t lastAngleEcd;      // 上一周期的编码器角度, 单位机械角度 0-8192
        int32_t round;             // 圈数
    } measurement;
    struct
    {
        float velocityRpm; // 电机速度, 单位: rpm
        float angle;       // 电机角度, 单位: 弧度
        float currentMA;   // 电机电流, 单位: 毫安
        float torqueNm;    // 电机扭矩, 单位: 牛米
    } physicalParam;
} DJIMotor_s;

/**
 * @brief 电机组
 * @note 电机组实际上是利用了大疆电机一拖四的特性，以降低总线负载
 */
typedef struct DJIMotorGroup
{
    DJIMotorBus_t ownerBus; // 该组所在的总线
    uint16_t id;            // 标识符，对应DJI电调反馈报文的标识符
    DJIMotor_t motorTable;  // 电机表
    uint8_t motorNum;       // 电机数量
} DJIMotorGroup_s;

/**
 * @brief 大疆电机总线，对应同一个CAN总线下的电机
 *
 */
typedef struct DJIMotorBus
{
    Device_t canHandler;         // 设备句柄
    DJIMotorGroup_t motorGroupTable;    // 电机组表
    uint8_t groupNum;           // 电机组数量
} DJIMotorBus_s;

#ifdef __cplusplus
}
#endif

#endif
