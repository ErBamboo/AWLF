#include "component/driver/pal_dev/pal_can_dev.h"
#include "core/awlf_cpu.h"
#include "algorithm/controller/pid.h"
#include "FreeRTOS.h" // TODO: 引入posix兼容层
#include "task.h"

#define GM6020_SEND_ID       (0x1FFU) // 发送标识符，当前表示电压发送
#define GM6020_ID            (1U)     // GM6020电机ID
#define GM6020_FEEDBACK_BASE (0x204U)
#define GM6020_FEEDBACK_ID   (GM6020_FEEDBACK_BASE + GM6020_ID) // 反馈报文ID

#ifndef PI
#define PI (3.1415f)
#endif

#define RPM_2_RADPS(rpm)      ((2.0f * (rpm) * PI) / 60.0f)     // rpm ——> rad/s
#define RADPS_2_RPM(radps)    ((60.0f * (radps)) / (2.0f * PI)) // rad/s ——> rpm

#define ECD_2_ANGLE(ecd)      ((ecd) * (360.0f / 8192.0f))  // 编码值 —— 角度转换
#define ANGLE_2_RAD(angle)    ((angle) * (PI / 180.0f))     // 角度 ——> rad
#define ECD_2_RAD(ecd)        ANGLE_2_RAD(ECD_2_ANGLE(ecd)) // 编码值 ——> rad

#define RPM_2_ECD(motor, rpm) ((int16_t)(((rpm) / (motor)->constVelPerVolt) * ((motor)->maxVoltageValue / 24.0f)))

typedef struct GM6020* GM6020_t;
typedef struct GM6020
{
    Device_t canHandler;
    uint8_t motorID;
    uint32_t sendID;
    uint32_t FeedbackID;
    float constVelPerVolt;    // 速度与电压的比例关系
    uint32_t maxVoltageValue; // 最大电压值 abs(25000)
    uint8_t settingCurrunt[8];
    uint8_t feedbackValue[8]; // 反馈值
    PidController_s speedPid;
    PidController_s anglePid;
    struct
    {
        float velocity;             // 速率，单位 rad/s
        int32_t totalRound;         // 总圈数（有向）
        float totalAngle;           // 总角度（有向）, 单位rad
        float angleSingleAngle;     // 单圈角度, 单位rad
        int16_t currunt;            // 电流值
        uint8_t temperature;        // 摄氏度
        uint16_t lastFeedbackAngle; // 上一周期的编码器角度, 单位机械角度 0-8192
        int16_t lastFeedbackSpeed;  // 上一周期的编码器速度, 单位rpm
    } statusValue_s;                // 状态量
    struct
    {
        uint16_t angle; // 机械角度 0-8191
        int16_t speed;  // 单位rpm
        int16_t currunt;
        uint8_t temperature; // 摄氏度
        uint8_t rvs;
    } __awlf_packed feedbackMeasurement_s; // 反馈量
    CanUserMsg_s feedbackMsg;
    CanUserMsg_s sendMsg;
    float currentTick;
} GM6020_s;

static void gm6020_update(GM6020_t gm6020)
{
    float alpha = 1.0f;
    // 反馈值更新
    gm6020->feedbackMeasurement_s.angle = ((gm6020->feedbackValue[0] << 8) & 0xFF00) | (gm6020->feedbackValue[1] & 0x00FF);
    gm6020->feedbackMeasurement_s.speed = ((gm6020->feedbackValue[2] << 8) & 0xFF00) | (gm6020->feedbackValue[3] & 0x00FF);
    gm6020->feedbackMeasurement_s.speed =
        alpha * gm6020->feedbackMeasurement_s.speed + (1.0 - alpha) * gm6020->statusValue_s.lastFeedbackSpeed;
    gm6020->statusValue_s.lastFeedbackSpeed = gm6020->feedbackMeasurement_s.speed; // 上一周期的编码器速度, 单位rpm

    gm6020->feedbackMeasurement_s.currunt = ((gm6020->feedbackValue[4] << 8) & 0xFF00) | (gm6020->feedbackValue[5] & 0x00FF);

    // 状态值更新
    gm6020->statusValue_s.velocity    = RPM_2_RADPS(gm6020->feedbackMeasurement_s.speed); // 速度
    gm6020->statusValue_s.temperature = gm6020->feedbackValue[6];                         // 温度

    // 过0处理
    if (gm6020->feedbackMeasurement_s.angle - gm6020->statusValue_s.lastFeedbackAngle > 4096)
        gm6020->statusValue_s.totalRound--;
    else if (gm6020->feedbackMeasurement_s.angle - gm6020->statusValue_s.lastFeedbackAngle < -4096)
        gm6020->statusValue_s.totalRound++;
    gm6020->statusValue_s.lastFeedbackAngle = gm6020->feedbackMeasurement_s.angle;
    // 角度更新
    gm6020->statusValue_s.angleSingleAngle = ECD_2_RAD(gm6020->feedbackMeasurement_s.angle);
    gm6020->statusValue_s.totalAngle =
        ECD_2_RAD(gm6020->feedbackMeasurement_s.angle + gm6020->statusValue_s.totalRound * 8192.0f);
}

static GM6020_s GM6020;
static const float speedPidParam[3] = {1.07f, 0.01f, 0.00f};
static const float anglePidParam[3] = {0.0f, 0.0f, 0.0f};

static void can_filter_callback(Device_t dev, void* param, size_t filterBank, size_t msgCount)
{
    GM6020_t gm6020 = (GM6020_t)param;
    device_read(dev, NULL, &gm6020->feedbackMsg, 1);
    gm6020_update(gm6020);
}

static void can_err_cb(Device_t dev, uint32_t errorCode, void* param, size_t paramSz)
{
}

static void can_tx_callback(Device_t dev, void* param, size_t paramsz)
{
    GM6020_t gm6020 = (GM6020_t)param;
    device_write(dev, NULL, &gm6020->sendMsg, 1);
}

static void gm6020_init(GM6020_t gm6020, Device_t canHandler)
{
    gm6020->canHandler          = canHandler;
    gm6020->motorID             = GM6020_ID;
    gm6020->sendID              = GM6020_SEND_ID;
    gm6020->FeedbackID          = GM6020_FEEDBACK_ID;
    gm6020->feedbackMsg.bank    = 0;
    gm6020->feedbackMsg.userBuf = (uint8_t*)&gm6020->feedbackValue;
    gm6020->currentTick         = 0.0f;

    gm6020->sendMsg.userBuf = (uint8_t*)gm6020->settingCurrunt;
    gm6020->sendMsg.dsc     = CAN_DATA_MSG_DSC_INIT(GM6020_SEND_ID, CAN_IDE_STD, 8);
    gm6020->constVelPerVolt = 13.33f;
    gm6020->maxVoltageValue = 25000.0f;
    pid_init(&gm6020->speedPid, PID_POSITIONAL_MODE, speedPidParam[0], speedPidParam[1], speedPidParam[2]);
    pid_set_dead_band(&gm6020->speedPid, 2.0f);
    pid_set_output_limit(&gm6020->speedPid, -gm6020->maxVoltageValue, gm6020->maxVoltageValue);
    pid_set_integral_limit(&gm6020->speedPid, 12500.0f);
    pid_set_variable_integral_thresholds(&gm6020->speedPid, 2000.0f, 15000.0f);

    pid_init(&gm6020->anglePid, PID_POSITIONAL_MODE, anglePidParam[0], anglePidParam[1], anglePidParam[2]);
    pid_set_dead_band(&gm6020->anglePid, 2.0f);
    pid_set_output_limit(&gm6020->anglePid, 0, 8191);

    // pid_set_derivative_first_enable(&gm6020->anglePid, false);
}

int16_t speedOut;
static void gm6020_set_speed(GM6020_t gm6020, float speed_rpm)
{
    // 转换为电机可识别的 ecd 单位
    int16_t nowSpeedEcd       = RADPS_2_RPM(gm6020->feedbackMeasurement_s.speed);
    int16_t setSpeedEcd       = RPM_2_ECD(gm6020, speed_rpm);
    gm6020->currentTick       = awlf_cpu_get_time_s();
    speedOut                  = (int16_t)pid_compute(&gm6020->speedPid, setSpeedEcd, nowSpeedEcd, gm6020->currentTick);
    gm6020->settingCurrunt[0] = (speedOut >> 8) & 0xFF;
    gm6020->settingCurrunt[1] = speedOut & 0xFF;
    device_write(gm6020->canHandler, NULL, &gm6020->sendMsg, 1);
}

volatile float speed = 50.0f; // 单位 rpm
void GM6020_task(void* param)
{
    AwlfRet_e ret = AWLF_OK;
    Device_t can  = device_find("can1");
    while (!can)
    {
    };

    /* CAN设备初始化 */
    ret = device_open(can, CAN_O_INT_RX | CAN_O_INT_TX);
    while (ret != AWLF_OK)
    {
    };
    CanFilterCfg_s FilterCfg = CAN_FILTER_CFG_INIT(0, CAN_FILTER_MODE_MASK, CAN_FILTER_ID_STD, GM6020_FEEDBACK_BASE + 1U, 0x200,
                                                   can_filter_callback, &GM6020);
    device_ctrl(can, CAN_CMD_SET_FILTER, &FilterCfg);
    device_set_param(can, &GM6020);
    device_set_err_cb(can, can_err_cb);
    device_set_write_cb(can, can_tx_callback);
    device_ctrl(can, CAN_CMD_START, NULL);
    gm6020_init(&GM6020, can);
    gm6020_set_speed(&GM6020, speed);
    while (1)
    {
    }
}

int main(void)
{
    TaskHandle_t task1;
    BaseType_t result1;

    result1 = xTaskCreate(GM6020_task, "GM6020_task", 512, NULL, 4, &task1);
    while (result1 != pdTRUE)
    {
    }
    awlf_cpu_init();

    vTaskStartScheduler();
    // 调度成功后不会跑到这里
    while (1)
    {
    }
    return 0;
}
