#include "core/awlf_cpu.h"
#include "core/awlf_def.h"

#include "gm6020.h"
#include "FreeRTOS.h" // TODO: 引入posix兼容层
#include "task.h"

static void can_err_cb(Device_t dev, uint32_t errorCode, void* param, size_t paramSz)
{
}

static void can_tx_callback(Device_t dev, void* param, size_t paramsz)
{
    GM6020_t gm6020 = (GM6020_t)param;
}

static void can_filter_callback(Device_t dev, void* param, size_t filterBank, size_t msgCount)
{
    GM6020_t gm6020 = (GM6020_t)param;
    device_read(dev, NULL, &gm6020->feedbackMsg, 1);
    gm6020_update(gm6020);
}

__dbg_param_def(float, speed) = 200.0f;
__dbg_param_def(float, angle) = 180.0f;
__dbg_param_def(float, dbgSpeedFbCoef) = 0.2f;
__dbg_param_def(float, dbgAccFbCoef) = 0.003f;
static GM6020_s GM6020;

void GM6020_task(void* param)
{
    AwlfRet_e ret = AWLF_OK;
    Device_t can = device_find("can1");
    while (!can) {};

    /* CAN设备初始化 */
    ret = device_open(can, CAN_O_INT_RX | CAN_O_INT_TX);
    while (ret != AWLF_OK) {};
    CanFilterCfg_s FilterCfg = CAN_FILTER_CFG_INIT(0, CAN_FILTER_MODE_MASK, CAN_FILTER_ID_STD, GM6020_FEEDBACK_BASE + 1U, 0x200,
                                                   can_filter_callback, &GM6020);

    device_ctrl(can, CAN_CMD_SET_FILTER, &FilterCfg);
    device_set_param(can, &GM6020);
    device_set_err_cb(can, can_err_cb);
    device_set_write_cb(can, can_tx_callback);
    device_ctrl(can, CAN_CMD_START, NULL);
    gm6020_init(&GM6020, can);

    TickType_t xTimeIncrement = 0.0;
    float t = 0.0f;
    float setAngle = 0.0f;
    float speedFb = 0.0f;
    float accFb = 0.0f;
    while (1)
    {
        xTimeIncrement = xTaskGetTickCount();
        t = xTimeIncrement / 1000.0f;

        setAngle = angle * sinf(2.0f * 3.1415f * t);
        speedFb = angle * cosf(2.0f * 3.1415f * (t + 1));
        accFb = angle * -sinf(2.0f * 3.1415f * (t + 1));

        gm6020_set_acc_feedback(&GM6020, accFb, dbgAccFbCoef);
        gm6020_set_speed_feedback(&GM6020, speedFb, dbgSpeedFbCoef);
        gm6020_set_single_angle(&GM6020, setAngle);
        vTaskDelayUntil(&xTimeIncrement, pdMS_TO_TICKS(1));
    }
}

int main(void)
{
    TaskHandle_t task1;
    BaseType_t result1;

    result1 = xTaskCreate(GM6020_task, "GM6020_task", 512, NULL, 4, &task1);
    while (result1 != pdTRUE) {}
    awlf_cpu_init();

    vTaskStartScheduler();
    // 调度成功后不会跑到这里
    while (1) {}
    return 0;
}
