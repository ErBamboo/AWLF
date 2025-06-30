#ifndef __CORE_DEF_H__
#define __CORE_DEF_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdatomic.h>


/*support files*/
#include "corelist.h"       /* klist - 移植自Linux，用于设备管理 */ 
#include "ringbuffer.h"     /* kfifo - 移植自Linux, 用于设备框架以及应用层的buffer设计，数据链路优化 */
#include "avltree.h"        // 自平衡二叉树 - 用于设计中断树 - 暂时抛弃
                            // 优点是遍历效率高，缺点是增加系统复杂度，难以调试

#define AWLF_NULL   ((void*)0)

#define __AWLF_VERSION__        "1.0.0"
#define __AWLF_AUTHOR__         "Bamboo"
#define __AWLF_COPYRIGHT__      "Copyright (c) 2025 Bamboo"
#define __AWLF_VERSION_INFO__   "AWLF Version: " __AWLF_VERSION__ " (" __AWLF_AUTHOR__ ")"



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


typedef uint32_t awlf_addr_t;
#define __SIZEOFPOINTER   (sizeof(awlf_addr_t))

/* 传输模式（通信类外设使用） */
typedef enum hw_trans_type 
{
    HW_POLL_TX = 0x01,      // 轮询发送
    HW_POLL_RX = 0x01 << 1, // 轮询接收
    HW_DMA_TX  = 0x01 << 2, // DMA发送
    HW_DMA_RX  = 0x01 << 3, // DMA接收
    HW_IRQ_TX  = 0x01 << 4, // 中断发送
    HW_IRQ_RX  = 0x01 << 5, // 中断接收
} hw_trans_type_e;

/* Ctrl 方法的 cmd参数  */
#define F_DEV_CTRL_CFG     0x00     // 设备配置
#define F_DEV_CTRL_CLR_INT 0x01
#define F_DEV_CTRL_SET_INT 0x02
#define F_DEV_CTRL_FLUSH   0x03     // 清空缓冲区

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
    REG_UNUSED    = 0,              // 未使用
    REG_DMA_TX    = 0x01 << 0,      // DMA发送
    REG_DMA_RX    = 0x01 << 1,      // DMA接收
    REG_IRQ_TX    = 0x01 << 2,      // 中断发送
    REG_IRQ_RX    = 0x01 << 3,      // 中断接收
    REG_STREAM    = 0x01 << 4,      // 流式
    REG_TIMER     = 0x01 << 5,      // 定时器
}regflag_e;

/* 设备打开方式(open方法的参数) */
typedef enum otype
{
    OTYPE_BLOCKING_RX       = 0x01 << 3,    // 阻塞式接收
    OTYPE_BLOCKING_TX       = 0x01 << 4,    // 阻塞式发送
    OTYPE_NON_BLOCKING_RX   = 0x01 << 5,    // 非阻塞式接收
    OTYPE_NON_BLOCKING_TX   = 0x01 << 6,    // 非阻塞式发送
    OTYPE_STREAM            = 0x01 << 7,    // 流式
} otype_e;

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
}dev_status_e;

typedef struct f_task_handle
{
    void*            task_handle;
    struct list_head task_list;
} f_task_handle_s;



#endif