#ifndef __SERIAL_INC__H
#define __SERIAL_INC__H

#include "stm32f4xx.h"
#include "drv_serial.h"
#include "bsp_serial.h"
#include "bsp_dma.h"

#define BSP_USE_SERIAL1_DMA_TWOBUF 1


/* USART1支持包 */
#ifdef USE_SERIAL_1
    #define SERIAL1_INSTANCE        USART1
    #define SERIAL1_IRQ             USART1_IRQn
    #define SERIAL1_CLK             RCC_APB2Periph_USART1
    #define SERIAL1_CLK_CMD         RCC_APB2PeriphClockCmd
    #define SERIAL1_GPIO_CLK        RCC_AHB1Periph_GPIOB
    #define SERIAL1_GPIO_CLK_CMD    RCC_AHB1PeriphClockCmd
    #define SERIAL1_GPIO_PORT       GPIOB
    #define SERIAL1_TX_PIN          GPIO_Pin_6
    #define SERIAL1_RX_PIN          GPIO_Pin_7
    #define SERIAL1_TX_AF           GPIO_AF_USART1
    #define SERIAL1_RX_AF           GPIO_AF_USART1
    #define SERIAL1_TX_SOURCE       GPIO_PinSource6
    #define SERIAL1_RX_SOURCE       GPIO_PinSource7

    /* FIFO SIZE */
    #define SERIAL1_TXFIFO_SIZE 1024
    #define SERIAL1_RXFIFO_SIZE 1024

    /*  USART1发送FIFO */
    #ifdef USE_SERIAL1_TXFIFO
        static uint8_t SERIAL1_TXBUF[SERIAL1_TXFIFO_SIZE];
        #define SERIAL1_TX_RB (ringbuf_s){          \
            .buf = SERIAL1_TXBUF,                      \
            .esize = sizeof(uint8_t),                   \
            .mask = SERIAL1_TXFIFO_SIZE - 1,            \
        }
        static serial_fifo_s SERIAL1_TX_FIFO ={           \
            .load_size = 0,                                \
            .rb = SERIAL1_TX_RB,                           \
        };
    #endif

    /*  USART1接收FIFO */
    #ifdef USE_SERIAL1_RXFIFO
        static uint8_t SERIAL1_RXBUF[SERIAL1_RXFIFO_SIZE];
        #define SERIAL1_RX_RB (ringbuf_s){          \
            .buf = SERIAL1_RXBUF,                      \
            .esize = sizeof(uint8_t),                   \
            .mask = SERIAL1_RXFIFO_SIZE - 1,            \
        }
        static serial_fifo_s SERIAL1_RX_FIFO ={           \
            .load_size = 0,                                \
            .rb = SERIAL1_RX_RB,                           \
        };
    #endif

    /* USART1发送DMA配置 */
    #ifdef USE_SERIAL1_DMA_TX
        #define SERIAL1_DMA_TX_CFG (stm32_dma_s) {              \
            .stream = DMA2_Stream7,\
            .cfg = {    \
                    .rcc = RCC_AHB1Periph_DMA2,\
                    .irq = DMA2_Stream7_IRQn,                  \
                    .itflag_tc = DMA_IT_TCIF7,          \
                    .init = {.DMA_Channel = DMA_Channel_4}\
                    },   \
            }     
        static stm32_dma_s SERIAL1_DMA_TX = SERIAL1_DMA_TX_CFG;
        #define USART1_DMA_Tx_Handler DMA2_Stream7_IRQHandler
    #endif

    /* USART1接收DMA配置 */
    #ifdef USE_SERIAL1_DMA_RX
        static uint8_t SERIAL1_DMA_RXBUF[SERIAL1_RXFIFO_SIZE];
        #if (BSP_USE_SERIAL1_DMA_TWOBUF)
            static uint8_t SERIAL1_DMA_DOUBLE_RXBUF[SERIAL1_RXFIFO_SIZE];
        #else
            #define SERIAL1_DMA_DOUBLE_RXBUF NULL
        #endif
        #define SERIAL1_DMA_RX_CFG (stm32_dma_s) {              \
                .stream = DMA2_Stream2,\
                .cfg = {    \
                        .rcc =  RCC_AHB1Periph_DMA2,\
                        .irq = DMA2_Stream2_IRQn,                  \
                        .itflag_ht = DMA_IT_HTIF2,                     \
                        .itflag_tc = DMA_IT_TCIF2, \
                        .init = {.DMA_Channel = DMA_Channel_4}     \
                    },                                           \
                .bufs = {   \
                        {   \
                            .buf = SERIAL1_DMA_RXBUF,          \
                            .bufsz = SERIAL1_RXFIFO_SIZE,       \
                        }   \
                    },\
                .active_idx = 0 \
            }
        static stm32_dma_s SERIAL1_RX_DMA = SERIAL1_DMA_RX_CFG;
        #define USART1_DMA_Rx_Handler DMA2_Stream2_IRQHandler 
    #endif


    /*  USART1初始化参数 */
    #define SERIAL1_BASE_INIT       (USART_InitTypeDef){    \
        .USART_BaudRate = 115200,                           \
        .USART_WordLength = USART_WordLength_8b,            \
        .USART_StopBits = USART_StopBits_1,                 \
        .USART_Parity = USART_Parity_No,                    \
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,        \
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None \
        }      
      
    /*  USART1中断初始化参数 */        
    #define SERIAL1_NVIC_INIT (NVIC_InitTypeDef){           \
        .NVIC_IRQChannel = SERIAL1_IRQ,                     \
        .NVIC_IRQChannelPreemptionPriority = 6,             \
        .NVIC_IRQChannelSubPriority = 0,                    \
        .NVIC_IRQChannelCmd = ENABLE                        \
        }      
                                      
    /*  USART1实例配置参数 */                                                                                         
    #define SERIAL1_BSP_CFG (stm32_serial_cfg_s){           \
        .name = "usart1",                                   \
        .regflags = REG_DMA_RX | REG_DMA_TX | REG_IRQ_TX | REG_IRQ_RX,   \
        .init = SERIAL1_BASE_INIT,                          \
    }  

    /* USART1实例化 */  
    #define SERIAL1_HANDLE (stm32_serial_s){                   \
        .cfg = SERIAL1_BSP_CFG,                             \
        .instance = SERIAL1_INSTANCE,                       \
        .serial = {                                         \
            .cfg = SERIAL_DEFAULT_CFG,                      \
        },                                                  \
    }                                                  
#endif /* USE_SERIAL_1 */


#endif
