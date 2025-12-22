#include "component/driver/device/motor/dji_motor.h"
#include "component/driver/pal/pal_can_dev.h"

#define DJI_MOTOR_BUS_NUM (2U)
#define DJI_MOTOR_NUM (1U)
#define DJI_MOTOR_GROUP_NUM (1U)


static void dji_motor_update(DJIMotor_t motor, uint8_t* buf)
{
    motor->measurement.lastAngleEcd = motor->measurement.angleEcd;
    motor->measurement.angleEcd = ((buf[0] << 8) & 0xFF00) | (buf[1] & 0x00FF);
    motor->measurement.velocity = ((buf[2] << 8) & 0xFF00) | (buf[3] & 0x00FF);
    motor->measurement.current = ((buf[4] << 8) & 0xFF00) | (buf[5] & 0x00FF);
    motor->measurement.temperature = buf[6];
    motor->errCode = buf[7];
}

/**
 * @brief 初始化大疆电调，主要处理ID设置
 * 
 * @param esc 电调句柄
 * @param cfg 电机配置
 * @note C610/C620不支持电流模式
 */
static void dji_esc_init(DJIMotorESC_t esc, DJIMotorCfg_t cfg)
{
    if(cfg->escType == DJI_ESC_C610 || cfg->escType == DJI_ESC_C620)
    {
        while(cfg->ctrlMode == DJI_MOTOR_CTRL_CURRENT){} // TODO: assert， C610/C620不支持电流模式
        esc->feedbackBasedId = 0x200;
        esc->sendId = (cfg->escId < 4) ? 0x200 : 0x1FF;
    }
    else if(cfg->escType == DJI_ESC_GM6020)
    {
        esc->feedbackBasedId = 0x204;
        if(cfg->ctrlMode == DJI_MOTOR_CTRL_VOLTAGE)
            esc->sendId = (cfg->escId < 4) ? 0x1FF : 0x2FF;
        else
            esc->sendId = (cfg->escId < 4) ? 0x1FE : 0x2FE;
    }
    else
        while(1){} // TODO: assert， 未知的电调类型
    esc->type = cfg->escType;
    esc->ctrlMode = cfg->ctrlMode;
    esc->escId = cfg->escId;
}

static void dji_can_filter_callbcak(Device_t Can, void* param, size_t filterBank, size_t msgCount)
{
    uint8_t buf[8];
    CanUserMsg_s motorMsg;
    uint8_t tableId = 0;
    uint8_t sz = 0;
    DJIMotorBus_t bus = (DJIMotorBus_t)param;
    for (uint8_t i = 0; i < msgCount; i++)
    {
        motorMsg.bank = filterBank;
        motorMsg.userBuf = buf;
        sz = device_read(Can, NULL, &motorMsg, 1);
        while(sz <= 0){} // TODO: assert， 读取到的报文大小错误
        
        // dji_motor_update()
    }
}

void dji_motor_init(DJIMotor_t Motor, DJIMotorCfg_t cfg)
{
    while (!Motor || !cfg || cfg->escId >= DJI_MOTOR_MAX_NUM_PER_BUS) {} // TODO: assert
    dji_esc_init(&Motor->esc, cfg);

}

void dji_motor_bus_init(DJIMotorBus_t bus, Device_t canHandler, uint16_t id, DJIMotor_t motorTable, uint8_t motorNum)
{
    while (!bus || !canHandler || motorTable || motorNum > DJI_MOTOR_MAX_NUM_PER_BUS) {} // TODO: assert
    bus->canHandler = canHandler;

    // CAN_CMD_SET_FILTER_AUTO_BANK下bank参数无效，随便填
    CanFilterCfg_s FilterCfg =
        CAN_FILTER_CFG_INIT(0, CAN_FILTER_MODE_MASK, CAN_FILTER_ID_STD, 0x1FF, 0x1FF, dji_can_filter_callbcak, bus);
    device_ctrl(canHandler, CAN_CMD_SET_FILTER_AUTO_BANK, &FilterCfg);
}
