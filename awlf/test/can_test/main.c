#include "component/driver/pal/pal_dev.h"
#include "aw_core/awlf_cpu.h"
#include "FreeRTOS.h" // TODO: 引入posix兼容层
#include "task.h"

#define CAN_MSG_BUF_LEN    8
#define CAN_TX_MSG_BUF_LEN 1000

typedef struct CanInfo* CanInfo_t;
typedef struct CanInfo
{
    CanUserMsg_s msg[CAN_MSG_BUF_LEN];
    uint8_t data[CAN_MSG_BUF_LEN][8];
} __awlf_packed CanInfo_s;

typedef struct CanTxInfo* CanTxInfo_t;
typedef struct CanTxInfo
{
    CanUserMsg_s msg[CAN_TX_MSG_BUF_LEN];
    uint8_t data[CAN_TX_MSG_BUF_LEN][8];
} __awlf_packed CanTxInfo_s;

static void can_info_init(CanUserMsg_t msg, uint32_t bank, uint8_t* data)
{
    msg->bank    = bank;
    msg->userBuf = data;
}

static void can_txinfo_init(CanUserMsg_t msg, uint8_t* data)
{
    msg->bank    = 0;
    msg->userBuf = data;
}

uint8_t cnt = 0;
static void can_filter_callback(Device_t dev, void* param, size_t filterBank, size_t msgCount)
{
    CanInfo_t info = (CanInfo_t)param;
    device_read(dev, NULL, &info->msg[cnt], msgCount);
    for (size_t i = 0; i < msgCount; i++)
    {
        info->msg[cnt + i].dsc.id =
            info->msg[cnt + i].dsc.id + 0x100; // CAN总线下不允许出现相同ID，因此这里加0x100避免和上位机ID冲突
    }
    device_write(dev, NULL, &info->msg[cnt], msgCount);
    cnt++;
    cnt %= CAN_MSG_BUF_LEN;
}

CanInfo_s canInfo = {0};
CanTxInfo_s canTxInfo;

void can_test_task(void* param)
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

    for (size_t i = 0; i < CAN_MSG_BUF_LEN; i++)
        can_info_init(&canInfo.msg[i], 0, canInfo.data[i]);

    CanFilterCfg_s FilterCfg =
        CAN_FILTER_CFG_INIT(0, CAN_FILTER_MODE_MASK, CAN_FILTER_ID_STD_EXT, 0x101, 0x1F0, can_filter_callback, (void*)&canInfo);
    device_ctrl(can, CAN_CMD_SET_FILTER, &FilterCfg);
    device_ctrl(can, CAN_CMD_START, NULL);

    while (1)
    {
        // while(cnt < CAN_TX_MSG_BUF_LEN)
        // {
        //     device_write(can, NULL, &canTxInfo.msg[cnt], 1);
        //     cnt++;
        // }
    }
}

int main(void)
{
    TaskHandle_t task1;
    BaseType_t result1;

    result1 = xTaskCreate(can_test_task, "CanTestTask", 512, NULL, 4, &task1);
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
