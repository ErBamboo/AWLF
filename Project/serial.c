#include "serial.h"

void serial_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){};
    USART_ClearFlag(USART1, USART_FLAG_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ClearFlag(USART1, USART_FLAG_TXE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

#include "f_task.h"
#include "f_completion.h"


// extern port_task_handle_t port_task2_handle;
// extern port_task_handle_t port_task1_handle;
// extern completion_s      serial_rx_comp;

// static uint8_t data_buf[10] = {1,2,3,4,5,6,7,8,9,10};
// static uint8_t pdata = 0;
// void USART1_IRQHandler(void)
// {
//     long isContextSwitchNeeded = 0;

//     if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
//         USART_ClearITPendingBit(USART1, USART_IT_TXE);
//         USART_SendData(USART1, data_buf[pdata++]);
//         if(pdata == 10) {
//             pdata = 0;
//             USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//             //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
//         }
//     }

//     if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {
//         //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//         USART_ClearITPendingBit(USART1, USART_IT_TC);
//     }

//     if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//         uint8_t data = USART_ReceiveData(USART1);
//         USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        
//         f_completion_done(&serial_rx_comp);
//     }
// }
