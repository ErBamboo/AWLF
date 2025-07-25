#ifndef __AWLF_DEF_H__
#define __AWLF_DEF_H__

#define AWLF_NULL   ((void*)0)

typedef enum {
    AWLF_FALSE = 0,
    AWLF_TRUE = !AWLF_FALSE
}awlf_bool_e;

typedef enum {
    AWLF_DISABLE = 0,
    AWLF_ENABLE = !AWLF_DISABLE
}awlf_action_e;

/* 返回值定义(标准错误码) */
typedef enum
{
    AWLF_OK = 0,               // 成功
    AWLF_ERROR,                // 通用错误
    AWLF_ERR_CONFLICT,         // 参数冲突
    AWLF_ERR_Irq,              // 中断错误
    AWLF_ERR_OVERFLOW,         // 溢出错误
    AWLF_ERROR_TIMEOUT,        // 超时错误
    AWLF_ERROR_DMA,            // DMA错误
    AWLF_ERROR_MEMORY,         // 内存错误
    AWLF_ERROR_PARAM,          // 参数错误
    AWLF_ERROR_BUSY,           // 忙错误
    AWLF_ERROR_EMPTY,          // 空转错误
    AWLF_ERROR_NOT_SUPPORT,    // 不支持的操作
} awlf_ret_t;

/* Ctrl 方法的 cmd参数  */
#define DEV_CTRL_CFG     0x00     // 设备配置
#define DEV_CTRL_CLR_INT 0x01
#define DEV_CTRL_SET_INT 0x02
#define DEV_CTRL_FLUSH   0x03     // 清空缓冲区

/* 设备类型 */
#define REG_COMMUNITY_DEV 0   // 通信类外设
#define REG_TIMER_DEV     1   // 定时器类外设
#define REG_USB_DEV       2   // USB类外设
#define REG_PWM_DEV       3   // PWM类外设
#define REG_I2C_DEV       4   // I2C类外设
#define REG_SPI_DEV       5   // SPI类外设
#define REG_CAN_DEV       6   // CAN类外设
#define REG_AD_DA_DEV     7   // ADC/DAC类外设

/* 设备设备注册参数 */ 
typedef enum
{           
    REG_PARAM_UNUSED    = 0,              // 未使用
    REG_PARAM_DMA_TX    = 0x01 << 0,      // DMA发送
    REG_PARAM_DMA_RX    = 0x01 << 1,      // DMA接收
    REG_PARAM_INT_TX    = 0x01 << 2,      // 中断发送
    REG_PARAM_INT_RX    = 0x01 << 3,      // 中断接收
    REG_PARAM_TIMER     = 0x01 << 5,      // 定时器
} regparam_e;

/* 设备打开方式(open方法的参数) */
typedef enum otype
{
    OPARAM_BLOCKING_RX       = 0x01 << 3,    // 阻塞式接收
    OPARAM_BLOCKING_TX       = 0x01 << 4,    // 阻塞式发送
    OPARAM_NON_BLOCKING_RX   = 0x01 << 5,    // 非阻塞式接收
    OPARAM_NON_BLOCKING_TX   = 0x01 << 6,    // 非阻塞式发送
} oparam_e;

/* 设备状态 */
typedef enum dev_status {
    DEV_STATUS_CLOSED   = 0x00,             // 设备关闭
    DEV_STATUS_REGED    = 0x01 << 0,        // 设备注册
    DEV_STATUS_OK       = 0x01 << 1,        // 设备正常
    DEV_STATUS_BUSY_RX  = 0x01 << 2,        // 设备接收中
    DEV_STATUS_BUSY_TX  = 0x01 << 3,        // 设备发送中
    DEV_STATUS_BUSY     = DEV_STATUS_BUSY_RX | DEV_STATUS_BUSY_TX,
    DEV_STATUS_TIMEOUT  = 0x01 << 4,        // 设备超时
    DEV_STATUS_ERR      = 0x01 << 5,        // 设备错误
    DEV_STATUS_INITED   = 0x01 << 6,        // 设备初始化完成
    DEV_STATUS_OPENED   = 0x01 << 8,        // 设备打开
} dev_status_e;

#endif  // __AWLF_DEF_H__