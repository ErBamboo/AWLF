#ifndef __BSP_H__
#define __BSP_H__

#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_serial.h"

#include "aw_core/awlf_cpu.h"
#include "stm32f4xx_hal.h"

extern uint32_t SystemCoreClock;

// TODO: 板级版本管理
#define __AWLF_BOARD_VERSION "1.0.0"
#define __AWLF_CPU_FREQ_MHZ  (SystemCoreClock / 1000000U)

#endif
