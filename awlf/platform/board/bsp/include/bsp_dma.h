#ifndef STM32_DMA_H
#define STM32_DMA_H

#include <stddef.h>
#include "stm32f4xx.h"

#define DMA_RCC_CMD(x, action)          RCC_AHB1PeriphClockCmd(x, action)
#define DMA_STREAM_ENABLE(x, action)    DMA_Cmd(x, action)

typedef DMA_Stream_TypeDef dma_type;
typedef struct stm32_dma_cfg
{
    IRQn_Type             irq;
    uint32_t              itflag_tc;
    uint32_t              itflag_ht;
    DMA_InitTypeDef       init;
    uint32_t              rcc;
} stm32_dma_cfg_s;

typedef struct dma_buf* dma_buf_t;
typedef struct dma_buf
{
    void*   buf;
    size_t  bufsz;
    size_t  last_counter;
}dma_buf_s;

typedef struct stm32_dma* stm32_dma_t;
typedef struct stm32_dma {
    dma_type*             stream;
    stm32_dma_cfg_s       cfg;
    dma_buf_s             bufs[2];
    uint8_t               active_idx;
} stm32_dma_s;

#endif
