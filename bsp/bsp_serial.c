#include "bsp_serial.h"
#include "serial_inc.h"


#define __USART_GET_DATA(handle)   ((handle)->instance->DR & (handle)->rx_mask)

typedef enum 
{
#ifdef USE_SERIAL_1
    SERIAL1_IDX,
#endif
#ifdef USE_SERIAL_2
    SERIAL2_IDX,
#endif
#ifdef USE_SERIAL_3
    SERIAL3_IDX,
#endif
#ifdef USE_SERIAL_4
    SERIAL4_IDX,
#endif
#ifdef USE_SERIAL_5
    SERIAL5_IDX,
#endif
#ifdef USE_SERIAL_6
    SERIAL6_IDX,
#endif
#ifdef USE_SERIAL_7
    SERIAL7_IDX,
#endif
#ifdef USE_SERIAL_8
    SERIAL8_IDX,
#endif
}serial_idx_e;


typedef enum
{
    DMA_HT,     // 半满
    DMA_TC,     // 全满
    IDLE,       // 接收空闲
}stm32_serial_rx_event;

static void _stm32_serial_init(stm32_serial_t handle)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_TypeDef*  GPIOx;
    uint8_t tx_gpio_source, rx_gpio_source;
    uint32_t tx_gpio_pin, rx_gpio_pin;
    uint8_t tx_gpio_af, rx_gpio_af;
    /* 1、开时钟 */
    switch ((uint32_t)handle->instance)
    {
#ifdef USE_SERIAL_1
        case (uint32_t)SERIAL1_INSTANCE:
            tx_gpio_source = SERIAL1_TX_SOURCE;
            rx_gpio_source = SERIAL1_RX_SOURCE;
            tx_gpio_pin = SERIAL1_TX_PIN;
            rx_gpio_pin = SERIAL1_RX_PIN;
            tx_gpio_af = SERIAL1_TX_AF;
            rx_gpio_af = SERIAL1_RX_AF;
            GPIOx = SERIAL1_GPIO_PORT;
            NVIC_InitStructure = SERIAL1_NVIC_INIT;
        break;
#endif
#ifdef USE_SERIAL_3
        case (uint32_t)SERIAL3_INSTANCE:
            tx_gpio_source = SERIAL3_TX_SOURCE;
            rx_gpio_source = SERIAL3_RX_SOURCE;
            tx_gpio_pin = SERIAL3_TX_PIN;
            rx_gpio_pin = SERIAL3_RX_PIN;
            tx_gpio_af = SERIAL3_TX_AF;
            rx_gpio_af = SERIAL3_RX_AF;
            GPIOx = SERIAL3_GPIO_PORT;
            NVIC_InitStructure = SERIAL3_NVIC_INIT;
        break;
#endif
    }
    GPIO_PinAFConfig(GPIOx, tx_gpio_source, tx_gpio_af);
    GPIO_PinAFConfig(GPIOx, rx_gpio_source, rx_gpio_af);
    GPIO_InitStructure.GPIO_Pin = tx_gpio_pin | rx_gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
    USART_Init(handle->instance, &handle->cfg.init);
    /* 小细节，防止开启串口就进入发送完成中断 */
    USART_ClearFlag(handle->instance, USART_FLAG_TC);
    USART_Cmd(handle->instance, ENABLE);
    while(USART_GetFlagStatus(handle->instance, USART_FLAG_TC) == RESET){};
    USART_ClearFlag(handle->instance, USART_FLAG_TC);
    NVIC_Init(&NVIC_InitStructure);
}

static void _stm32_dma_cfg(stm32_serial_t handle, uint32_t dma_flag)
{
    stm32_dma_t dma_handle;
    DMA_InitTypeDef* DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;    

    if(dma_flag == REG_DMA_TX)
    {
        dma_handle = handle->tx_dma;
        DMA_InitStructure = &dma_handle->cfg.init;
        DMA_InitStructure->DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure->DMA_Mode = DMA_Mode_Normal;
        USART_DMACmd(handle->instance, USART_DMAReq_Tx, ENABLE);
        DMA_ITConfig(dma_handle->stream, DMA_IT_TC, ENABLE);
   }
    else if(dma_flag == REG_DMA_RX)
    {
        dma_handle = handle->rx_dma;
        DMA_InitStructure = &dma_handle->cfg.init;
        DMA_InitStructure->DMA_Memory0BaseAddr = (uint32_t)dma_handle->bufs[0].buf;
        DMA_InitStructure->DMA_BufferSize = dma_handle->bufs[0].bufsz;
        DMA_InitStructure->DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure->DMA_Mode = DMA_Mode_Circular;
        USART_DMACmd(handle->instance, USART_DMAReq_Rx, ENABLE);
        DMA_ClearFlag(dma_handle->stream, dma_handle->cfg.itflag_tc);
        DMA_ClearFlag(dma_handle->stream, dma_handle->cfg.itflag_ht);
        DMA_ITConfig(dma_handle->stream, DMA_IT_TC, ENABLE);
        DMA_ITConfig(dma_handle->stream, DMA_IT_HT, ENABLE);
        USART_ClearFlag(handle->instance, USART_FLAG_IDLE);
        USART_ITConfig(handle->instance, USART_IT_IDLE, ENABLE);
    }
    else
        while(1){};
    DMA_InitStructure->DMA_PeripheralBaseAddr = (uint32_t)&handle->instance->DR;
    DMA_InitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    DMA_InitStructure->DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    DMA_InitStructure->DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure->DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure->DMA_Priority = DMA_Priority_High;
    
    NVIC_InitStructure.NVIC_IRQChannel = dma_handle->cfg.irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    DMA_Init(dma_handle->stream, DMA_InitStructure);
    NVIC_Init(&NVIC_InitStructure);
    if(dma_flag == REG_DMA_RX)
    {
        if(dma_handle->bufs[1].buf)
        {
            DMA_DoubleBufferModeConfig(dma_handle->stream, (uint32_t)dma_handle->bufs[1].buf, DMA_Memory_0);
            DMA_DoubleBufferModeCmd(dma_handle->stream, ENABLE);
        }
        DMA_Cmd(dma_handle->stream, ENABLE);
    }
}

static void _stm32_dma_transmit(stm32_serial_t handle, const uint8_t* buff, size_t size)
{
    stm32_dma_t dma_handle = handle->tx_dma;
    if(DMA_GetCmdStatus(dma_handle->stream) == ENABLE) return;
    DMA_InitTypeDef* DMA_InitStructure;
    DMA_InitStructure = &dma_handle->cfg.init;
    DMA_InitStructure->DMA_Memory0BaseAddr = (uint32_t)buff;
    DMA_InitStructure->DMA_BufferSize = size;
    DMA_Cmd(dma_handle->stream, DISABLE);
    DMA_Init(dma_handle->stream, DMA_InitStructure);
    DMA_SetCurrDataCounter(dma_handle->stream, size);
    DMA_Cmd(dma_handle->stream, ENABLE);
}

/** @brief  serial interface 
 *  @param  serial: serial handle
 *  @param  cfg: serial configuration
 *  @retval AWLF_OK: success
 *  @retval AWLF_ERROR_PARAM: error parameter
 */
static awlf_ret_t stm32_serial_configure(drv_serial_t serial, serial_cfg_t cfg)
{
    if(!serial) return AWLF_ERROR_PARAM;
    stm32_serial_t handle = (stm32_serial_t)serial;
    
    handle->cfg.init.USART_BaudRate = cfg->baudrate;
    handle->cfg.init.USART_WordLength = (cfg->parity != PARITY_NONE) ? USART_WordLength_9b : USART_WordLength_8b;
    handle->cfg.init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    switch(cfg->stopbits)
    {
        case STOP_BITS_0_5:
            handle->cfg.init.USART_StopBits = USART_StopBits_0_5;   break;
        case STOP_BITS_1:
            handle->cfg.init.USART_StopBits = USART_StopBits_1;     break;
        case STOP_BITS_1_5:
            handle->cfg.init.USART_StopBits = USART_StopBits_1_5;   break;
        case STOP_BITS_2:
            handle->cfg.init.USART_StopBits = USART_StopBits_2;     break;
        default:
            handle->cfg.init.USART_StopBits = USART_StopBits_1;     break;
    }

    switch(cfg->parity)
    {
        case PARITY_NONE:
            handle->cfg.init.USART_Parity = USART_Parity_No;   break;
        case PARITY_EVEN:
            handle->cfg.init.USART_Parity = USART_Parity_Even; break;
        case PARITY_ODD:
            handle->cfg.init.USART_Parity = USART_Parity_Odd;  break;
        default:
            handle->cfg.init.USART_Parity = USART_Parity_No;   break;
    }

    switch(cfg->flow_ctrl)
    {
        case FLOW_CTRL_NONE:
            handle->cfg.init.USART_HardwareFlowControl = USART_HardwareFlowControl_None; break;
        case FLOW_CTRL_RTS:
            handle->cfg.init.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;  break;
        case FLOW_CTRL_CTS:
            handle->cfg.init.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;  break;
        case FLOW_CTRL_CTSRTS:
            handle->cfg.init.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;  break;
        default:
            handle->cfg.init.USART_HardwareFlowControl = USART_HardwareFlowControl_None; break;
    }
    handle->rx_mask = serial_get_rxmask(serial);
    USART_Init(handle->instance, &handle->cfg.init);
    return AWLF_OK;
}

/**
 * @brief Send a byte of data through the serial interface.
 * 
 * @param serial Pointer to the serial handle.
 * @param data Byte of data to send.
 * @return awlf_ret_t Status of the operation.
 */
static awlf_ret_t stm32_serial_putByte(drv_serial_t serial, uint8_t data)
{
    stm32_serial_t handle = (stm32_serial_t)serial;
    handle->instance->DR = data & (uint16_t)0x01FF;
    while(USART_GetFlagStatus(handle->instance, USART_FLAG_TXE) == RESET);
    return AWLF_OK;
}

/**
 * @brief Receive a byte of data from the serial interface.
 * @param serial Pointer to the serial handle.
 * @return uint8_t Received byte of data.
 */
static uint8_t stm32_serial_getByte(drv_serial_t serial)
{
    stm32_serial_t handle = (stm32_serial_t)serial;
    while(USART_GetFlagStatus(handle->instance, USART_FLAG_RXNE) == RESET);
    return (uint8_t)__USART_GET_DATA(handle);
}

static awlf_ret_t stm32_serial_control(drv_serial_t serial, uint32_t cmd, void* arg) 
{
    stm32_serial_t handle;
    while(!serial)
    {
        // @todo: assert
    }
    handle = (stm32_serial_t)serial;

    switch (cmd) {
        case F_DEV_CTRL_CFG:
        {
            uint32_t _arg = (uint32_t)arg;
            if(_arg & (REG_DMA_TX | REG_DMA_RX))
            {
                _stm32_dma_cfg(handle, _arg);
                // USART_ClearFlag(handle->instance, USART_FLAG_TC);         // 清除发送完成标志
                // USART_ITConfig(handle->instance, USART_IT_TC, ENABLE);    // 发送完成中断使能   
            }
            else if(_arg == REG_IRQ_RX)
            {
                stm32_serial_control(serial, F_DEV_CTRL_SET_INT, (void*)_arg);   // 递归调用ctrl方法，开启发送中断
            }   
            USART_ITConfig(handle->instance, USART_IT_ERR, ENABLE);
            /* 中断发送不需要使能 */
        }
        break;

        case F_DEV_CTRL_SET_INT:
        {
            uint32_t _arg = (uint32_t)arg;
            if(_arg ==  REG_IRQ_TX)
                USART_ITConfig(handle->instance, USART_IT_TXE, ENABLE);
            else if(_arg == REG_IRQ_RX)
                USART_ITConfig(handle->instance, USART_IT_RXNE, ENABLE);
        }
        break;

        case F_DEV_CTRL_CLR_INT:
        {
            uint32_t _arg = (uint32_t)arg;
            if(_arg == REG_IRQ_TX)
                USART_ITConfig(handle->instance, USART_IT_TXE, DISABLE);
            else if(_arg == REG_IRQ_RX)
                USART_ITConfig(handle->instance, USART_IT_RXNE, DISABLE);
            USART_ITConfig(handle->instance, USART_IT_ERR, DISABLE);
        }
        break;

        default:
            return AWLF_ERROR_PARAM;
    }
    return AWLF_OK;
}

static size_t stm32_serial_transmit(drv_serial_t serial, const uint8_t* data, size_t length) {
    stm32_serial_t handle;
    if(!serial || !data || !length) return 0;
    handle = (stm32_serial_t)serial;

    if(handle->cfg.regflags & REG_DMA_TX)
    {
        _stm32_dma_transmit(handle, data, length);
    }
    else
    {
        stm32_serial_control(serial, F_DEV_CTRL_SET_INT, (void*)REG_IRQ_TX);
    }

    return length;
}

static serial_interface_s serial_interface = {
    .configure  = stm32_serial_configure,
    .putByte    = stm32_serial_putByte,
    .getByte    = stm32_serial_getByte,
    .control    = stm32_serial_control,
    .transmit   = stm32_serial_transmit,
};

static stm32_serial_s handler[] = {
#ifdef USE_SERIAL_1
    SERIAL1_HANDLE,
#endif
#ifdef USE_SERIAL_2
    SERIAL2_HANDLE,
#endif
#ifdef USE_SERIAL_3
    SERIAL3_HANDLE,
#endif
#ifdef USE_SERIAL_4
    SERIAL4_HANDLE,
#endif
#ifdef USE_SERIAL_5
    SERIAL5_HANDLE,
#endif
#ifdef USE_SERIAL_6
    SERIAL6_HANDLE,
#endif

};

static void bsp_get_cfg(void)
{
    #ifdef USE_SERIAL_1
        /* FIFO */
        #ifdef USE_SERIAL1_TXFIFO
            handler[SERIAL1_IDX].serial.cfg.txbufsz = SERIAL1_TXFIFO_SIZE;
            handler[SERIAL1_IDX].serial.__priv.tx_fifo = &SERIAL1_TX_FIFO;
        #endif
        #ifdef USE_SERIAL1_RXFIFO
            handler[SERIAL1_IDX].serial.cfg.rxbufsz = SERIAL1_RXFIFO_SIZE;
            handler[SERIAL1_IDX].serial.__priv.rx_fifo = &SERIAL1_RX_FIFO;
        #endif
        /* TXDMA */
        #ifdef USE_SERIAL1_DMA_TX
            handler[SERIAL1_IDX].tx_dma = &SERIAL1_DMA_TX;
            DMA_RCC_CMD(SERIAL1_DMA_TX.cfg.rcc, ENABLE);
        #endif
        /* RXDMA */
        #ifdef USE_SERIAL1_DMA_RX
            handler[SERIAL1_IDX].rx_dma = &SERIAL1_RX_DMA;
            DMA_RCC_CMD(SERIAL1_RX_DMA.cfg.rcc, ENABLE);
            /*  DMA DoubleBufferMode */
            #ifdef BSP_USE_SERIAL1_DMA_TWOBUF
                SERIAL1_RX_DMA.bufs[1].buf = SERIAL1_DMA_DOUBLE_RXBUF;
                SERIAL1_RX_DMA.bufs[1].bufsz = SERIAL1_RXFIFO_SIZE;
            #endif
        #endif
        SERIAL1_CLK_CMD(SERIAL1_CLK, ENABLE);
        SERIAL1_GPIO_CLK_CMD(SERIAL1_GPIO_CLK, ENABLE);
    #endif

    #ifdef USE_SERIAL_3
        /* FIFO */
        #ifdef USE_SERIAL3_TXFIFO
            handler[SERIAL3_IDX].serial.cfg.txbufsz = SERIAL3_TXFIFO_SIZE;
            handler[SERIAL3_IDX].serial.__priv.tx_fifo = &SERIAL3_TX_FIFO;
        #endif
        #ifdef USE_SERIAL3_RXFIFO
            handler[SERIAL3_IDX].serial.cfg.rxbufsz = SERIAL3_RXFIFO_SIZE;
            handler[SERIAL3_IDX].serial.__priv.rx_fifo = &SERIAL3_RX_FIFO;
        #endif
        /* TXDMA */
        #ifdef USE_SERIAL3_DMA_TX
            handler[SERIAL3_IDX].tx_dma = &SERIAL3_DMA_TX;
            DMA_RCC_CMD(SERIAL3_DMA_TX.cfg.rcc, ENABLE);
        #endif
        /* RXDMA */
        #ifdef USE_SERIAL3_DMA_RX
            handler[SERIAL3_IDX].rx_dma = &SERIAL3_RX_DMA;
            DMA_RCC_CMD(SERIAL3_RX_DMA.cfg.rcc, ENABLE);
            /*  DMA DoubleBufferMode */
            #ifdef BSP_USE_SERIAL3_DMA_TWOBUF
                SERIAL3_RX_DMA.bufs[1].buf = SERIAL3_DMA_DOUBLE_RXBUF;
                SERIAL3_RX_DMA.bufs[1].bufsz = SERIAL3_RXFIFO_SIZE;
            #endif
        #endif
        SERIAL3_CLK_CMD(SERIAL3_CLK, ENABLE);
        SERIAL3_GPIO_CLK_CMD(SERIAL3_GPIO_CLK, ENABLE);
    #endif
}

void bsp_serial_init(void)
{
    bsp_get_cfg();
    for (uint8_t i = 0; i < sizeof(handler)/sizeof(handler[0]); i++)
    {
        handler[i].serial.interface = &serial_interface;
        serial_register(&handler[i].serial, handler[i].cfg.name, NULL, handler[i].cfg.regflags);
        _stm32_serial_init(&handler[i]);
    }
}

static void _stm32_dma_tx_isr(stm32_serial_t handle);
static void _stm32_rx_isr(stm32_serial_t handle, stm32_serial_rx_event event);

#ifdef USE_SERIAL_1
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
        serial_hw_isr(&handler[SERIAL1_IDX].serial, SERIAL_EVENT_INT_TXDONE, NULL, 0);
    }
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        serial_hw_isr(&handler[SERIAL1_IDX].serial, SERIAL_EVENT_INT_RXDONE, &data, 1);
    }
    if(USART_GetITStatus(USART1,USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        if(handler[SERIAL1_IDX].cfg.regflags & REG_DMA_RX)
            _stm32_rx_isr(&handler[SERIAL1_IDX], IDLE);
    }
}

// 串口1发送DMA服务函数
void USART1_DMA_Tx_Handler(void)
{
    stm32_serial_t handle = &handler[SERIAL1_IDX];
    if(DMA_GetITStatus(handle->tx_dma->stream, handle->tx_dma->cfg.itflag_tc) != RESET)
    {
        DMA_ClearITPendingBit(handle->tx_dma->stream, handle->tx_dma->cfg.itflag_tc);
        _stm32_dma_tx_isr(handle);
    }
}

void USART1_DMA_Rx_Handler(void)
{
    stm32_serial_t handle = &handler[SERIAL1_IDX];
    if(DMA_GetITStatus(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_ht) != RESET)
    {
        DMA_ClearITPendingBit(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_ht);
        _stm32_rx_isr(handle, DMA_HT);
    }

    if(DMA_GetITStatus(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_tc) != RESET)
    {
        DMA_ClearITPendingBit(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_tc);
        _stm32_rx_isr(handle, DMA_TC);
    }
}
#endif

#ifdef USE_SERIAL_3
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
        USART_ClearITPendingBit(USART3, USART_IT_TXE);
        serial_hw_isr(&handler[SERIAL3_IDX].serial, SERIAL_EVENT_INT_TXDONE, NULL, 0);
    }
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART3);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        serial_hw_isr(&handler[SERIAL3_IDX].serial, SERIAL_EVENT_INT_RXDONE, &data, 1);
    }
    if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART3);
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        if(handler[SERIAL3_IDX].cfg.regflags & REG_DMA_RX)
            _stm32_rx_isr(&handler[SERIAL3_IDX], IDLE);
    }
}

// 串口1发送DMA服务函数
void USART3_DMA_Tx_Handler(void)
{
    stm32_serial_t handle = &handler[SERIAL3_IDX];
    if(DMA_GetITStatus(handle->tx_dma->stream, handle->tx_dma->cfg.itflag_tc) != RESET)
    {
        DMA_ClearITPendingBit(handle->tx_dma->stream, handle->tx_dma->cfg.itflag_tc);
        _stm32_dma_tx_isr(handle);
    }
}
void USART3_DMA_Rx_Handler(void)
{
    stm32_serial_t handle = &handler[SERIAL3_IDX];
    if(DMA_GetITStatus(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_ht) != RESET)
    {
        DMA_ClearITPendingBit(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_ht);
        _stm32_rx_isr(handle, DMA_HT);
    }

    if(DMA_GetITStatus(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_tc) != RESET)
    {
        DMA_ClearITPendingBit(handle->rx_dma->stream, handle->rx_dma->cfg.itflag_tc);
        _stm32_rx_isr(handle, DMA_TC);
    }
}
#endif

static void _stm32_dma_tx_isr(stm32_serial_t handle)
{
    size_t tx_size;
    stm32_dma_t dma;
    dma = handle->tx_dma;
    tx_size = dma->cfg.init.DMA_BufferSize - DMA_GetCurrDataCounter(dma->stream);   // 通常是0，保险起见算一下
    serial_hw_isr(&handle->serial, SERIAL_EVENT_DMATXDONE, NULL, tx_size);
}

static void _stm32_rx_isr(stm32_serial_t handle, stm32_serial_rx_event event)
{
    stm32_dma_t dma = handle->rx_dma;
    size_t recv_size;
    size_t dma_rxcount;
    void* active_membuf;
    uint8_t now_buf_idx = dma->active_idx;
    if(event == DMA_HT || event == IDLE)
    {
        dma_rxcount = dma->bufs[now_buf_idx].bufsz - DMA_GetCurrDataCounter(dma->stream);
        recv_size = dma_rxcount - dma->bufs[now_buf_idx].last_counter;
    }
    else if(event == DMA_TC)
    {
        recv_size = dma->bufs[now_buf_idx].bufsz - dma->bufs[now_buf_idx].last_counter;
        dma_rxcount = 0;        // 该行给dma_rxcount赋0，最终目的是为了使last_count复位
        if(dma->bufs[1].buf)
            dma->active_idx ^= 1;
    }
    active_membuf = dma->bufs[now_buf_idx].buf;
    active_membuf = active_membuf + dma->bufs[now_buf_idx].last_counter;
    dma->bufs[now_buf_idx].last_counter = dma_rxcount;
    serial_hw_isr(&handle->serial, SERIAL_EVENT_DMARXDONE, active_membuf, recv_size);
}
