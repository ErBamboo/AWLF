#ifndef __HAL_SERIAL_H__
#define __HAL_SERIAL_H__


#include <stdint.h>
#include "core/awlf_device.h"
#include "itc/completion.h"
#include "data_struct/ringbuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct serial_cfg* serial_cfg_t;
typedef struct serial_interface* serial_interface_t;
typedef struct hal_serial* hal_serial_t;

/*  configure the serial */
#define USE_SERIAL_1 
// #define USE_SERIAL_2 
// #define USE_SERIAL_3
// #define USE_SERIAL_4
// #define USE_SERIAL_5
// #define USE_SERIAL_6

/* config serial dma */
#define USE_SERIAL1_DMA_TX
#define USE_SERIAL1_DMA_RX
// #define USE_SERIAL2_DMA_TX
// #define USE_SERIAL2_DMA_RX
#define USE_SERIAL3_DMA_TX
#define USE_SERIAL3_DMA_RX
// #define USE_SERIAL4_DMA_TX
// #define USE_SERIAL4_DMA_RX
// #define USE_SERIAL5_DMA_TX
// #define USE_SERIAL5_DMA_RX
// #define USE_SERIAL6_DMA_TX
// #define USE_SERIAL6_DMA_RX

#define USE_SERIAL1_TXFIFO
#define USE_SERIAL1_RXFIFO
// #define USE_SERIAL2_TXFIFO
// #define USE_SERIAL2_RXFIFO
#define USE_SERIAL3_TXFIFO
#define USE_SERIAL3_RXFIFO
// #define USE_SERIAL4_TXFIFO
// #define USE_SERIAL4_RXFIFO
// #define USE_SERIAL5_TXFIFO
// #define USE_SERIAL5_RXFIFO
// #define USE_SERIAL6_TXFIFO
// #define USE_SERIAL6_RXFIFO
 

/* 数据位，由一些辅助运算的magic number 构成*/
typedef enum data_bits {
    DATA_BITS_5 = 0x4000F,      // 5位数据位
    DATA_BITS_6 = 0x5001F,      // 6位数据位
    DATA_BITS_7 = 0x6003F,      // 7位数据位
    DATA_BITS_8 = 0x7007F,      // 8位数据位
    DATA_BITS_9 = 0x800FF,      // 9位数据位
} data_bits_e;

/* 停止位 */
typedef enum stop_bits {
    STOP_BITS_0_5 = 0x00,
    STOP_BITS_1   = 0x01,
    STOP_BITS_1_5 = 0x02,
    STOP_BITS_2   = 0x03,
} stop_bits_e;

/* 校验位 */
typedef enum parity_e {
    PARITY_NONE = 0x00,
    PARITY_ODD  = 0x01,
    PARITY_EVEN = 0x02,
} parity_e;

/* 硬件流控 */
typedef enum flow_ctrl {
    FLOW_CTRL_NONE   = 0x00,
    FLOW_CTRL_RTS    = 0x01 << 0,
    FLOW_CTRL_CTS    = 0x01 << 1,
    FLOW_CTRL_CTSRTS = FLOW_CTRL_RTS | FLOW_CTRL_CTS,
} flow_ctrl_e;

/* 串口中断事件 */
typedef enum serial_event {
    SERIAL_EVENT_INT_TXDONE = 0x01, // 中断发送
    SERIAL_EVENT_INT_RXDONE = 0x02, // 中断接收
    SERIAL_EVENT_DMATXDONE  = 0x03,     // DMA发送完成
    SERIAL_EVENT_DMARXDONE  = 0x04,     // DMA接收完成
    SERIAL_EVENT_ERR        = 0x05,     // 错误中断
} serial_event_e;

/* 串口工作模式，轮询为默认模式 */
typedef enum {
    SERIAL_DMA_TX = 0x01,
    SERIAL_DMA_RX = 0x01 << 1,
    SERIAL_INT_TX = 0x01 << 2,
    SERIAL_INT_RX = 0x01 << 3,
} serial_work_mode_e;

typedef struct serial_cfg {
    uint32_t    baudrate;      // 波特率
    data_bits_e databits;  // 数据位
    stop_bits_e stopbits : 2;  // 停止位
    parity_e    parity   : 2;  // 校验位
    flow_ctrl_e flow_ctrl : 2; // 硬件流控
    uint32_t    reserved : 26;  // 保留位
    uint32_t    txbufsz;       // 发送缓冲区大小
    uint32_t    rxbufsz;       // 接收缓冲区大小
} serial_cfg_s;

#define SERIAL_MIN_TX_BUFSZ 128
#define SERIAL_MIN_RX_BUFSZ 128

/* 默认配置 */
#define SERIAL_DEFAULT_CFG            \
    (serial_cfg_s)                    \
    {                                 \
        .baudrate  = 115200,          \
        .databits  = DATA_BITS_8,     \
        .stopbits  = STOP_BITS_1,     \
        .parity    = PARITY_NONE,     \
        .flow_ctrl = FLOW_CTRL_NONE,  \
        .txbufsz   = SERIAL_MIN_TX_BUFSZ, \
        .rxbufsz   = SERIAL_MIN_RX_BUFSZ  \
    }

typedef struct serial_interface {
    awlf_ret_t (*configure)(hal_serial_t serial, serial_cfg_t cfg);
    awlf_ret_t (*putByte)(hal_serial_t serial, uint8_t data);
    uint8_t (*getByte)(hal_serial_t serial);
    awlf_ret_t (*control)(hal_serial_t serial, uint32_t cmd, void* arg);
    size_t (*transmit)(hal_serial_t serial, const uint8_t* data, size_t length);
} serial_interface_s;

typedef struct serial_fifo* serial_fifo_t;
typedef struct serial_fifo {
    ringbuf_s     rb;             // 环形缓冲区
    completion_s  cpt;            // 完成信号
    uint16_t      load_size;      // 串口正在加载的数据长度（即正在发送、接收的数据长度）
}serial_fifo_s;

typedef struct serial_priv* serial_priv_t;
typedef struct serial_priv {
    serial_fifo_t  tx_fifo;
    serial_fifo_t  rx_fifo;
} serial_priv_s;

/* 原则上，所有__priv成员都不允许被外部修改 */
typedef struct hal_serial {
    device_s            parent;
    serial_interface_t  interface;
    serial_cfg_s        cfg;
    serial_priv_s       __priv;
} hal_serial_s;

awlf_ret_t serial_register(hal_serial_t serial, char* name, void* handle, uint32_t regflag);
awlf_ret_t serial_hw_isr(hal_serial_t serial, serial_event_e event, void* arg, size_t arg_size);
awlf_ret_t serial_init_fifo(hal_serial_t serial, serial_fifo_t fifo, uint8_t* buf, size_t bufsz);
inline serial_fifo_t serial_get_rxfifo(hal_serial_t serial);
uint32_t serial_get_rxmask(hal_serial_t serial);

#ifdef __cplusplus
}
#endif

#endif
