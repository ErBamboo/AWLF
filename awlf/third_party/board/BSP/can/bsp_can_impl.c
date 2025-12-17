/*
 * @Description: CAN模块实现文件
 * @date 2025-11-10
 * @author 余浩
 */
#include "bsp_can.h"
#include <string.h>

// 以下行注释的作用是 对"clang-format off"和"clang-format on"之间的代码关闭格式化
// clang-format off
static uint32_t gBs1Table[CAN_TSEG1_MAX] =
{
    CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ,
    CAN_BS1_4TQ, CAN_BS1_5TQ, CAN_BS1_6TQ,
    CAN_BS1_7TQ, CAN_BS1_8TQ, CAN_BS1_9TQ,
    CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
    CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, 
    CAN_BS1_16TQ,
};

static uint32_t gBs2Table[CAN_TSEG2_MAX] =
{
    CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ,
    CAN_BS2_4TQ, CAN_BS2_5TQ, CAN_BS2_6TQ,
    CAN_BS2_7TQ, CAN_BS2_8TQ,
};

// clang-format on

static inline uint32_t bsp_can_bs1_trans(CanBS1_e bs1)
{
    while (bs1 >= CAN_TSEG1_MAX || bs1 < 0) {} // TODO: assert
    return gBs1Table[bs1];
}

static inline uint32_t bsp_can_bs2_trans(CanBS2_e bs2)
{
    while (bs2 >= CAN_TSEG2_MAX || bs2 < 0) {} // TODO: assert
    return gBs2Table[bs2];
}

static uint32_t bsp_can_sjw_trans(CanSjw_e sjw)
{
    uint32_t ret = 0;
    switch (sjw)
    {
        case CAN_SYNCJW_1TQ: ret = CAN_SJW_1TQ; break;
        case CAN_SYNCJW_2TQ: ret = CAN_SJW_2TQ; break;
        case CAN_SYNCJW_3TQ: ret = CAN_SJW_3TQ; break;
        case CAN_SYNCJW_4TQ: ret = CAN_SJW_4TQ; break;
        default:
            while (1) {}; // TODO: assert
            break;
    }
    return ret;
}

static AwlfRet_e bsp_can_set_filter(CAN_HandleTypeDef* hcan, CanFilterCfg_t cfg)
{
    CAN_FilterTypeDef FilterConfig;
    FilterConfig.FilterBank           = cfg->bank;
    FilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterActivation     = ENABLE;
    FilterConfig.FilterFIFOAssignment = (cfg->bank % 2 == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    // STM32F4的CAN为主从CAN结构，CAN1为主CAN，CAN2为从CAN，共用1组28个滤波器，本驱动程序选择对半分
    FilterConfig.SlaveStartFilterBank = 14;
    // 配置滤波器参数
    if (cfg->workMode == CAN_FILTER_MODE_MASK)
    {
        uint32_t id;
        uint32_t mask;
        FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        if (cfg->idType == CAN_FILTER_ID_STD) // 仅标准帧
        {
            id                            = cfg->id << 5;
            mask                          = cfg->mask << 5;
            FilterConfig.FilterIdHigh     = id;
            FilterConfig.FilterIdLow      = 0;
            FilterConfig.FilterMaskIdHigh = mask;
            FilterConfig.FilterMaskIdLow  = CAN_ID_EXT;
        }
        else
        {
            id                        = cfg->id << 3;
            mask                      = cfg->mask << 3;
            FilterConfig.FilterIdHigh = (id >> 16) & 0xffff;
            FilterConfig.FilterIdLow  = (id & 0xffff);

            FilterConfig.FilterMaskIdHigh = (mask >> 16) & 0xffff;
            FilterConfig.FilterMaskIdLow  = (mask & 0xffff);

            if (cfg->idType == CAN_FILTER_ID_EXT) // 仅拓展帧
            {
                FilterConfig.FilterIdLow |= CAN_ID_EXT;
                FilterConfig.FilterMaskIdLow |= CAN_ID_EXT;
            }
            else // 标准帧 + 拓展帧
            {
                FilterConfig.FilterIdLow = (id & 0xffff);
                FilterConfig.FilterMaskIdLow &= ~CAN_ID_EXT;
            }
        }
    }
    else
    {
        FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        if (cfg->idType == CAN_FILTER_ID_STD)
        {
            FilterConfig.FilterIdHigh = cfg->id << 5;
            FilterConfig.FilterIdLow  = 0;
        }
        else
        {
            if (cfg->idType == CAN_FILTER_ID_EXT)
                FilterConfig.FilterIdLow = ((cfg->id << 3) & 0xffff) | CAN_ID_EXT;
            else
                FilterConfig.FilterIdLow = ((cfg->id << 3) & 0xffff);
            FilterConfig.FilterIdHigh = ((cfg->id << 3) >> 16) & 0xffff;
        }
    }
    if (HAL_CAN_ConfigFilter(hcan, &FilterConfig) != HAL_OK)
        return AWLF_ERROR;
    return AWLF_OK;
}

static uint8_t bsp_can_get_tx_mailbox_free_level(CAN_HandleTypeDef* hcan, uint32_t mailboxBank)
{
    uint32_t freelevel = 0U;
    if (hcan->State == HAL_CAN_STATE_READY || hcan->State == HAL_CAN_STATE_LISTENING)
    {
        switch (mailboxBank)
        {
            case 0:
                if ((hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
                    freelevel++;
                break;
            case 1:
                if ((hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
                    freelevel++;
                break;
            case 2:
                if ((hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
                    freelevel++;
                break;
            default:
                while (1) {}; // TODO: assert
                break;
        }
    }
    return freelevel;
}

/**
 * @brief 1添加一条待发送的CAN消息到指定的邮箱
 *
 * @param hcan CAN句柄
 * @param pTxHeader 待发送的CAN消息头
 * @param pData 待发送的CAN消息数据
 * @param mailboxBank 邮箱索引，0~2
 * @return HAL_StatusTypeDef HAL_OK表示成功，HAL_ERROR表示失败
 */
static HAL_StatusTypeDef bsp_can_add_tx_msg(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* pTxHeader, uint8_t* pData,
                                            uint32_t mailboxBank)
{
    HAL_CAN_StateTypeDef state = hcan->State;
    if ((state != HAL_CAN_STATE_READY) || (state != HAL_CAN_STATE_LISTENING))
    {
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
        return HAL_ERROR;
    }
    uint32_t canTMEx = (1U << (CAN_TSR_TME0_Pos + mailboxBank));
    uint32_t tsr     = READ_REG(hcan->Instance->TSR);

    if ((tsr & canTMEx) == 0)
    {
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;
        return HAL_ERROR;
    }

    uint32_t               transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
    CAN_TxMailBox_TypeDef* mailbox         = &hcan->Instance->sTxMailBox[transmitmailbox];

    mailbox->TIR &= CAN_TI0R_TXRQ;
    if (pTxHeader->IDE == CAN_ID_STD)
        mailbox->TIR |= pTxHeader->StdId << CAN_TI0R_STID_Pos | pTxHeader->RTR;
    else if (pTxHeader->IDE == CAN_ID_EXT)
        mailbox->TIR |= pTxHeader->ExtId << CAN_TI0R_EXID_Pos | pTxHeader->RTR | CAN_ID_EXT;

    mailbox->TDTR = pTxHeader->DLC;
    if (pTxHeader->TransmitGlobalTime == ENABLE)
    {
        SET_BIT(hcan->Instance->sTxMailBox[mailboxBank].TDTR, CAN_TDT0R_TGT);
    }
    mailbox->TDLR = (pData[0] << CAN_TDL0R_DATA0_Pos) | (pData[1] << CAN_TDL0R_DATA1_Pos) | (pData[2] << CAN_TDL0R_DATA2_Pos) |
                    (pData[3] << CAN_TDL0R_DATA3_Pos);
    mailbox->TDHR = (pData[4] << CAN_TDH0R_DATA4_Pos) | (pData[5] << CAN_TDH0R_DATA5_Pos) | (pData[6] << CAN_TDH0R_DATA6_Pos) |
                    (pData[7] << CAN_TDH0R_DATA7_Pos);
    SET_BIT(hcan->Instance->sTxMailBox[mailboxBank].TIR, CAN_TI0R_TXRQ);
    return HAL_OK;
}

// 波特率配置表，CAN时钟42MHz，不同主频需要修改该表
static CanTimeCfg_s BspCanBitTimeTable[] = {
    {CAN_BAUD_10K, 300, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_20K, 150, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_50K, 60, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_100K, 30, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_125K, 24, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_250K, 12, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_500K, 6, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_800K, 4, {CAN_TSEG1_8TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
    {CAN_BAUD_1M, 3, {CAN_TSEG1_9TQ, CAN_TSEG2_4TQ, CAN_SYNCJW_2TQ}},
};

static CanTimeCfg_t bsp_can_time_cfg_matched(CanBaudRate_e baud)
{
    for (int i = 0; i < sizeof(BspCanBitTimeTable) / sizeof(BspCanBitTimeTable[0]); i++)
    {
        if (BspCanBitTimeTable[i].baudRate == baud)
        {
            return &BspCanBitTimeTable[i];
        }
    }
    while (1) {}; // TODO: assert
}

static AwlfRet_e bsp_can_configure(HalCanHandler_t Can, CanCfg_t cfg)
{
    BspCan_t           bsp_can = (BspCan_t)Can->parent.handle;
    CAN_HandleTypeDef* hcan    = (CAN_HandleTypeDef*)bsp_can;
    CanTimeCfg_t       TimeCfg = bsp_can_time_cfg_matched(cfg->normalTimeCfg.baudRate);
    CanBS1_e           bs1     = TimeCfg->bitTimeCfg.bs1;
    CanBS2_e           bs2     = TimeCfg->bitTimeCfg.bs2;
    CanSjw_e           sjw     = TimeCfg->bitTimeCfg.syncJumpWidth;
    hcan->Init.Prescaler       = TimeCfg->psc;
    switch (cfg->workMode)
    {
        case CAN_WORK_NORMAL: hcan->Init.Mode = CAN_MODE_NORMAL; break;
        case CAN_WORK_LOOPBACK: hcan->Init.Mode = CAN_MODE_LOOPBACK; break;
        case CAN_WORK_SILENT: hcan->Init.Mode = CAN_MODE_SILENT; break;
        case CAN_WORK_SILENT_LOOPBACK: hcan->Init.Mode = CAN_MODE_SILENT_LOOPBACK; break;
        default:
            while (1) {}; // TODO: assert
            break;
    }
    hcan->Init.SyncJumpWidth      = bsp_can_sjw_trans(sjw);
    hcan->Init.TimeSeg1           = bsp_can_bs1_trans(bs1);
    hcan->Init.TimeSeg2           = bsp_can_bs2_trans(bs2);
    hcan->Init.ReceiveFifoLocked  = (cfg->functionalCfg.rxFifoLockMode == 1) ? ENABLE : DISABLE;
    hcan->Init.TimeTriggeredMode  = (cfg->functionalCfg.timeTriggeredMode == 1) ? ENABLE : DISABLE;
    hcan->Init.AutoBusOff         = (cfg->functionalCfg.autoBusOff == 1) ? ENABLE : DISABLE;
    hcan->Init.AutoRetransmission = (cfg->functionalCfg.autoRetransmit == 1) ? ENABLE : DISABLE;
    hcan->Init.AutoWakeUp         = (cfg->functionalCfg.autoWakeUp == 1) ? ENABLE : DISABLE;
    if (HAL_CAN_Init(hcan) != HAL_OK)
    {
        while (1) {}; // TODO: assert
    }
    HAL_CAN_ActivateNotification(hcan, CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING |
                                           CAN_IT_LAST_ERROR_CODE);
    return AWLF_OK;
}

static AwlfRet_e bsp_can_control(HalCanHandler_t Can, uint32_t cmd, void* arg)
{
    if (!Can || !Can->parent.handle)
        return AWLF_ERROR_PARAM;

    CAN_HandleTypeDef* hcan = (CAN_HandleTypeDef*)Can->parent.handle;
    AwlfRet_e          ret  = AWLF_OK;

    switch (cmd)
    {
        case CAN_CMD_GET_STATUS:
        {
            CanErrCounter_t errCounter = (CanErrCounter_t)arg;
            uint32_t        errReg     = READ_REG(hcan->Instance->ESR);
            errCounter->txErrCnt       = (errReg >> 16) & 0xFF;
            errCounter->rxErrCnt       = (errReg >> 24);
            ret                        = AWLF_OK;
        }
        break;
        case CAN_CMD_START:
            // 启动CAN通信
            if (HAL_CAN_Start(hcan) != HAL_OK)
                ret = AWLF_ERROR;
            break;

        case CAN_CMD_CFG:
            // 配置CAN
            ret = bsp_can_configure(Can, (CanCfg_t)arg);
            break;

        case CAN_CMD_SUSPEND:
            // 暂停CAN通信
            if (HAL_CAN_Stop(hcan) != HAL_OK)
                ret = AWLF_ERROR;
            break;

        case CAN_CMD_RESUME:
            // 恢复CAN通信
            if (HAL_CAN_Start(hcan) != HAL_OK)
                ret = AWLF_ERROR;
            break;

        case CAN_CMD_SET_IOTYPE:
            // 设置CAN IO类型，使能中断
            while (arg == NULL) {}; // TODO: assert
            uint32_t io_type = *(uint32_t*)arg;
            if (io_type == CAN_REG_INT_TX)
            {
                // 使能发送中断
                uint32_t txIntEvents = CAN_IT_TX_MAILBOX_EMPTY;
                HAL_CAN_ActivateNotification(hcan, txIntEvents);
            }
            else if (io_type == CAN_REG_INT_RX)
            {
                uint32_t rxIntEvents = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                       CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL;
                HAL_CAN_ActivateNotification(hcan, rxIntEvents);
            }
            break;

        case CAN_CMD_CLR_IOTYPE:
        {
            while (arg == NULL) {}; // TODO: assert
            // 清除CAN IO类型，禁用中断
            uint32_t io_type = *(uint32_t*)arg;
            if (io_type == CAN_REG_INT_TX)
            {
                // 失能发送中断
                HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
            }
            else if (io_type == CAN_REG_INT_RX)
            {
                uint32_t rxIntEvents = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | // 接收中断
                                       CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |         // 接收溢出
                                       CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL;                // 接收FIFO满
                HAL_CAN_DeactivateNotification(hcan, rxIntEvents);
            }
        }
        break;

        case CAN_CMD_CLOSE:
            // 关闭CAN设备
            if (HAL_CAN_Stop(hcan) != HAL_OK)
                ret = AWLF_ERROR;
            // TODO: 解初始化，关闭时钟，禁用中断
            break;

        case CAN_CMD_FLUSH:
            // 清空缓存 - 对于STM32 CAN，需要清空接收FIFO
            hcan->Instance->RF0R |= CAN_RF0R_RFOM0;
            hcan->Instance->RF1R |= CAN_RF1R_RFOM1;
            break;

        case CAN_CMD_SET_FILTER:
            // 设置滤波器
            while (!arg) {}; // TODO: assert
            {
                CanFilterCfg_t filter_cfg = (CanFilterCfg_t)arg;
                while ((hcan->Instance == CAN1) && filter_cfg->bank >= 14) {}; // TODO: assert
                while ((hcan->Instance == CAN2) && filter_cfg->bank >= 28) {}; // TODO: assert
                ret = bsp_can_set_filter(hcan, filter_cfg);
            }
            break;

        default: ret = AWLF_ERROR_PARAM; break;
    }

    return ret;
}

static AwlfRet_e bsp_can_recv_msg(HalCanHandler_t Can, CanUserMsg_t msg, int32_t rxfifo_bank)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             data[8];

    CAN_HandleTypeDef* hcan = (CAN_HandleTypeDef*)Can->parent.handle;
    AwlfRet_e          ret  = AWLF_OK;

    // 检查接收FIFO是否为空
    uint32_t fifo_fill_level = HAL_CAN_GetRxFifoFillLevel(hcan, rxfifo_bank);
    if (fifo_fill_level == 0)
        return AWLF_ERROR_EMPTY;

    // 从指定FIFO读取消息
    if (HAL_CAN_GetRxMessage(hcan, rxfifo_bank, &rx_header, data) != HAL_OK)
        return AWLF_ERROR;

    if (!msg) // 若msg为空，说明框架层RxFifo为空且不采取覆写策略，直接返回溢出错误
        return AWLF_ERR_OVERFLOW;

    // 填充用户消息结构
    msg->dsc.id      = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
    msg->dsc.idType  = (rx_header.IDE == CAN_IDE_STD) ? CAN_IDE_STD : CAN_IDE_EXT;
    msg->dsc.msgType = (rx_header.RTR == CAN_RTR_DATA) ? CAN_MSG_TYPE_DATA : CAN_MSG_TYPE_REMOTE;
    msg->dsc.dataLen = rx_header.DLC; // 在经典CAN中，STM32的DLC与框架中的dataLen是一致的（0 <= dataLen <= 8)
    // 设置滤波器编号
    msg->bank          = rx_header.FilterMatchIndex;
    msg->dsc.timeStamp = rx_header.Timestamp;
    memcpy(msg->userBuf, data, msg->dsc.dataLen);
    return AWLF_OK;
}

static AwlfRet_e bsp_can_send_msg(HalCanHandler_t Can, CanUserMsg_t msg)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t            stm32Mailbox;
    if (!Can || !Can->parent.handle || !msg)
        return AWLF_ERROR_PARAM;

    CAN_HandleTypeDef* hcan = (CAN_HandleTypeDef*)Can->parent.handle;

    // 检查发送邮箱是否已满
    uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
    if (free_level == 0)
    {
        msg->bank = -1;
        return AWLF_ERR_OVERFLOW;
    }

    // 准备发送头
    tx_header.StdId              = msg->dsc.id;
    tx_header.ExtId              = msg->dsc.id;
    tx_header.IDE                = (msg->dsc.idType == CAN_IDE_EXT) ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR                = (msg->dsc.msgType == CAN_MSG_TYPE_REMOTE) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC                = msg->dsc.dataLen;
    tx_header.TransmitGlobalTime = DISABLE; // 当前架构暂时不支持
    // 准备发送数据
    uint8_t data[8];
    if (msg->userBuf != NULL && msg->dsc.dataLen > 0)
    {
        memcpy(data, msg->userBuf, msg->dsc.dataLen);
    }
    // 自动选择发送邮箱并发送消息
    uint32_t txMailboxBank = 0;
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &txMailboxBank) != HAL_OK)
    {
        msg->bank = -1;
        return AWLF_ERROR;
    }

    // 填充发送邮箱索引
    switch (txMailboxBank)
    {
        case CAN_TX_MAILBOX0: msg->bank = 0; break;
        case CAN_TX_MAILBOX1: msg->bank = 1; break;
        case CAN_TX_MAILBOX2: msg->bank = 2; break;
    }
    return AWLF_OK;
}

static CanHwInterface_s gCanHwInterface = {
    .configure        = bsp_can_configure,
    .control          = bsp_can_control,
    .recv_msg         = bsp_can_recv_msg,
    .send_msg_mailbox = bsp_can_send_msg,
};

BspCan_s gBspCan[] = {
#ifdef USE_CAN1
    BSP_CAN_STATIC_INIT(CAN1, "can1", CAN1_REG_PARAMS),
#endif
#ifdef USE_CAN2
    BSP_CAN_STATIC_INIT(CAN2, "can2", CAN2_REG_PARAMS),
#endif
};

static void bsp_can_pre_init(void)
{
    CAN_HandleTypeDef* hcan = NULL;
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#ifdef USE_CAN1
    hcan = &gBspCan[BSP_CAN1_IDX].handle;
    __HAL_RCC_CAN1_CLK_ENABLE();
    if (__HAL_RCC_GPIOD_IS_CLK_DISABLED())
        __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0); // 优先级根据需要调整，考虑到RM机器人中CAN总线的重要性，这里给0
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
#endif
#ifdef USE_CAN2
    hcan = &gBspCan[BSP_CAN2_IDX].handle;
    __HAL_RCC_CAN2_CLK_ENABLE();
    if (__HAL_RCC_GPIOB_IS_CLK_DISABLED())
        __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_CAN_DeInit(hcan);
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
#endif
}

void bsp_can_register(void)
{
    uint8_t   cnt = sizeof(gBspCan) / sizeof(gBspCan[0]);
    AwlfRet_e ret = AWLF_OK;
    for (int i = 0; i < cnt; i++)
    {
        gBspCan[i].parent.hwInterface      = &gCanHwInterface;
        gBspCan[i].parent.adapterInterface = hal_can_get_classic_adapter_interface();
        ret = hal_can_register(&gBspCan[i].parent, gBspCan[i].name, &gBspCan[i], gBspCan[i].regparams);
        while (ret != AWLF_OK) {}; // TODO: assert
    }
    bsp_can_pre_init();
}
