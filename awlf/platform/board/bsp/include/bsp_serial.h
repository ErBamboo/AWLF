#ifndef __STM32_SERIAL_H__
#define __STM32_SERIAL_H__

#include "stm32f4xx.h"
#include "hal/device/hal_serial.h"
#include "bsp_dma.h"

typedef struct stm32_serial_cfg
{
    char*      name;
    regflag_e  regflags;
    USART_InitTypeDef   init;
} stm32_serial_cfg_s;

typedef struct stm32_serial* stm32_serial_t;
typedef struct stm32_serial
{
    hal_serial_s   serial;
    USART_TypeDef* instance;    // 串口实例
    stm32_serial_cfg_s cfg;     
    stm32_dma_t     tx_dma;
    stm32_dma_t     rx_dma;
    uint32_t        rx_mask;
} stm32_serial_s;

void bsp_serial_init(void);

#endif
