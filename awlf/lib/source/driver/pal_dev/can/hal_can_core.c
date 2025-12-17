#include "component/driver/pal_dev/pal_can_dev.h"
#include "core/awlf_interrupt.h"
#include "core/awlf_def.h"
#include <string.h>

static void _can_status_timer_cb(TimerHandle_t xTimer)
{
    HalCanHandler_t    Can           = (HalCanHandler_t)pvTimerGetTimerID(xTimer);
    CanStatusManager_t StatusManager = &Can->statusManager;
    // 检查CAN状态
    Can->hwInterface->control(Can, CAN_CMD_GET_STATUS, (void*)&StatusManager->errCounter);
    size_t canTxErrCnt = StatusManager->errCounter.txErrCnt;
    size_t canRxErrCnt = StatusManager->errCounter.rxErrCnt;
    if (canRxErrCnt < 127 && canTxErrCnt < 127)
        Can->statusManager.nodeErrStatus = CAN_NODE_STATUS_ACTIVE;
    else if (canRxErrCnt > 127 || canTxErrCnt > 127)
        Can->statusManager.nodeErrStatus = CAN_NODE_STATUS_PASSIVE;
    else if (canTxErrCnt > 255)
        Can->statusManager.nodeErrStatus = CAN_NODE_STATUS_BUSOFF;
    device_err_cb(&Can->parent, CAN_ERR_EVENT_BUS_STATUS, Can->statusManager.nodeErrStatus);
}

/*
 * @brief 初始化CAN接收FIFO
 * @param Can CAN句柄
 * @param msgNum 接收消息缓存数量
 * @retval  错误码(AwlfRet_e)   描述
 *          AWLF_OK             成功
 *          AWLF_ERROR_MEMORY   内存分配失败
 */
static AwlfRet_e _can_fifo_init(HalCanHandler_t Can, CanMsgFifo_t canFifo, uint32_t msgNum, uint8_t isOverWrite)
{
    AwlfRet_e ret = AWLF_OK;
    INIT_LIST_HEAD(&canFifo->freeList);
    canFifo->msgBuffer = Can->adapterInterface->msgbuffer_alloc(&canFifo->freeList, msgNum);
    while (canFifo->msgBuffer == NULL) {};
    // 初始化链表
    INIT_LIST_HEAD(&canFifo->usedList);
    canFifo->freeCount   = msgNum;
    canFifo->isOverwrite = isOverWrite;
    return ret;
}

static void _can_rxfifo_deinit(CanMsgFifo_t rxfifo)
{
    if (rxfifo->msgBuffer != NULL)
    {
        vPortFree(rxfifo->msgBuffer);
        rxfifo->msgBuffer = NULL;
    }
    rxfifo->freeCount = 0;
    INIT_LIST_HEAD(&rxfifo->freeList);
    INIT_LIST_HEAD(&rxfifo->usedList);
}

/*
 * @brief 初始化CAN接收过滤器
 * @param RxHandler CAN接收句柄
 * @param filterNum 接收过滤器数量
 * @retval  错误码(AwlfRet_e)   描述
 *          AWLF_OK             成功
 *          AWLF_ERROR_MEMORY   内存分配失败
 */
static AwlfRet_e _can_filter_init(CanRxHandler_t RxHandler, uint32_t filterNum)
{
    uint32_t filterSz;
    if (filterNum > 0)
    {
        filterSz               = filterNum * sizeof(CanFilter_s);
        RxHandler->filterTable = (CanFilter_t)pvPortMalloc(filterSz);
        if (RxHandler->filterTable == NULL)
        {
            // TODO: ASSERT
            return AWLF_ERROR_MEMORY;
        }
        memset(RxHandler->filterTable, 0, filterSz);
        for (uint32_t i = 0; i < filterNum; i++)
        {
            RxHandler->filterTable[i].cfg.bank = i;
            INIT_LIST_HEAD(&RxHandler->filterTable[i].msgMatchedList);
        }
    }
    else
    {
        RxHandler->filterTable = NULL;
    }
    return AWLF_OK;
}

static void _can_filter_deinit(CanRxHandler_t RxHandler)
{
    if (RxHandler->filterTable != NULL)
    {
        vPortFree(RxHandler->filterTable);
        RxHandler->filterTable = NULL;
    }
}

/*
 * @brief 初始化CAN接收句柄
 * @param Can CAN句柄
 * @param filterNum 接收过滤器数量
 * @param msgNum 接收消息缓存数量
 * @retval  错误码(AwlfRet_e)   描述
 *          AWLF_OK             成功
 *          AWLF_ERROR_BUSY     句柄已初始化
 *          AWLF_ERROR_MEMORY   内存分配失败
 */
static AwlfRet_e _can_rxhandler_init(HalCanHandler_t Can, uint32_t iotype, uint32_t filterNum, uint32_t msgNum)
{
    CanRxHandler_t RxHandler;
    uint32_t       regIoType;
    uint32_t       isOparamValid;
    AwlfRet_e      ret = AWLF_OK;
    // 至少初始化一个滤波器
    while (filterNum <= 0 || msgNum <= 0 || !Can->adapterInterface->msgbuffer_alloc) {}; // TODO: ASSERT

    // 检查IO类型是否有效
    regIoType     = device_get_regparams(&Can->parent) & DEVICE_REG_RXTYPE_MASK;
    isOparamValid = (iotype & regIoType);
    if (!isOparamValid)
    {
        // TODO: ASSERT "Invalid IO type"
        return AWLF_ERROR_PARAM;
    }

    RxHandler = &Can->rxHandler;
    if (RxHandler->filterTable != NULL || RxHandler->rxFifo.msgBuffer != NULL)
        return AWLF_ERROR_BUSY; // TODO:LOG

    ret = _can_fifo_init(Can, &RxHandler->rxFifo, msgNum, Can->cfg.functionalCfg.isRxOverwrite);
    if (ret != AWLF_OK)
        return ret; // TODO: ASSERT
    ret = _can_filter_init(RxHandler, filterNum);
    if (ret != AWLF_OK)
    {
        vPortFree(RxHandler->rxFifo.msgBuffer);
        RxHandler->rxFifo.msgBuffer = NULL;
        // TODO: ASSERT
        return ret;
    }
    Can->hwInterface->control(Can, CAN_CMD_SET_IOTYPE, (void*)&iotype);
    return ret;
}

static AwlfRet_e _can_status_manager_init(HalCanHandler_t Can)
{
    if (Can->statusManager.statusTimer != NULL)
        return AWLF_ERROR_BUSY;
    CanStatusManager_t StatusManager;
    char*              name = device_get_name(&Can->parent);
    StatusManager           = &Can->statusManager;
    StatusManager->statusTimer =
        xTimerCreate(name, pdMS_TO_TICKS(Can->cfg.statusCheckTimeout), pdTRUE, (void*)Can, _can_status_timer_cb);
    BaseType_t ret = xTimerStart(StatusManager->statusTimer, pdMS_TO_TICKS(Can->cfg.statusCheckTimeout));
    if (ret != pdPASS)
    {
        // TODO: ASSERT
        return AWLF_ERROR_TIMEOUT;
    }
    return AWLF_OK;
}

static void _can_status_manager_deinit(HalCanHandler_t Can)
{
    CanStatusManager_t StatusManager = &Can->statusManager;
    if (StatusManager->statusTimer != NULL)
    {
        xTimerDelete(StatusManager->statusTimer, pdMS_TO_TICKS(Can->cfg.statusCheckTimeout));
        StatusManager->statusTimer = NULL;
    }
}

__dbg_param_def(CanMailbox_t, dbgMailbox[3]) = {0};

static AwlfRet_e _can_txhandler_init(HalCanHandler_t Can, uint32_t iotype, size_t mailboxNum, uint32_t txMsgNum)
{
    CanTxHandler_t TxHandler;
    uint32_t       regIoType;
    uint32_t       isOparamValid;
    AwlfRet_e      ret = AWLF_OK;
    while (mailboxNum <= 0 || txMsgNum <= 0) {}; // TODO: ASSERT

    // 检查IO类型是否有效
    regIoType     = device_get_regparams(&Can->parent) & DEVICE_REG_TXTYPE_MASK;
    isOparamValid = (regIoType & iotype);
    if (!isOparamValid)
    {
        // TODO: ASSERT "Invalid IO type"
        return AWLF_ERROR_PARAM;
    }

    // 初始化FIFO
    TxHandler          = &Can->txHandler;
    size_t txMailBoxsz = Can->cfg.mailboxNum * sizeof(CanMailbox_s);
    ret                = _can_fifo_init(Can, &TxHandler->txFifo, txMsgNum, Can->cfg.functionalCfg.isTxOverwrite);
    while (ret != AWLF_OK) {}; // TODO: ASSERT
    // 初始化Mailbox
    TxHandler->pMailboxes = (CanMailbox_t)pvPortMalloc(txMailBoxsz);
    while (TxHandler->pMailboxes == NULL) {}; // TODO: ASSERT
    memset(TxHandler->pMailboxes, 0, txMailBoxsz);

    INIT_LIST_HEAD(&TxHandler->mailboxList);
    for (uint32_t i = 0; i < Can->cfg.mailboxNum; i++)
    {
        TxHandler->pMailboxes[i].bank = i;
        dbgMailbox[i]                 = &TxHandler->pMailboxes[i];
        INIT_LIST_HEAD(&TxHandler->pMailboxes[i].list);
        list_add_tail(&TxHandler->pMailboxes[i].list, &TxHandler->mailboxList);
    }
    Can->hwInterface->control(Can, CAN_CMD_SET_IOTYPE, (void*)&iotype);
    return ret;
}

static void _can_txhandler_deinit(CanTxHandler_t TxHandler)
{
    if (TxHandler->txFifo.msgBuffer != NULL)
    {
        vPortFree(TxHandler->txFifo.msgBuffer);
        TxHandler->txFifo.msgBuffer = NULL;
    }
    TxHandler->txFifo.freeCount = 0;
    INIT_LIST_HEAD(&TxHandler->txFifo.freeList);
    INIT_LIST_HEAD(&TxHandler->txFifo.usedList);
}

/*
 * @brief CAN 接收模块解初始化
 * @param RxHandler CAN接收句柄
 * @retval 无
 */
static void _can_rxhandler_deinit(CanRxHandler_t RxHandler)
{
    if (RxHandler->filterTable != NULL)
    {
        vPortFree(RxHandler->filterTable);
        RxHandler->filterTable = NULL;
    }
    if (RxHandler->rxFifo.msgBuffer != NULL)
    {
        vPortFree(RxHandler->rxFifo.msgBuffer);
        RxHandler->rxFifo.msgBuffer = NULL;
    }
}

/*
 * @brief 将CAN消息链表项中的数据拷贝到CAN用户消息中
 * @param msgList CAN消息链表项
 * @param pUserRxMsg CAN用户消息指针
 * @retval 无
 */
static inline void _can_container_copy_to_usermsg(CanMsgList_t MsgList, CanUserMsg_t pUserRxMsg)
{
    MsgList->userMsg.userBuf = pUserRxMsg->userBuf; // 防止框架层的userBuf覆盖原有的用户内存指针
    *pUserRxMsg              = MsgList->userMsg;
    // 拷贝数据到用户缓冲区
    memcpy((void*)pUserRxMsg->userBuf, (void*)MsgList->container, MsgList->userMsg.dsc.dataLen);
    MsgList->userMsg.userBuf = MsgList->container; // 恢复框架层的userBuf指针
}

/**
 * @brief 从CAN FIFO中获取一个空闲链表项
 * @param Fifo CAN FIFO
 * @return 错误码
 * @retval 1. CAN_ERR_NONE          成功
 * @retval 2. CAN_ERR_SOFT_FIFO_OVERFLOW 接收FIFO溢出
 *
 * @note 在覆写模式下，返回CAN_ERR_SOFT_FIFO_OVERFLOW时MsgList将指向被取出的链表项
 *       否则MsgList将指向NULL
 * @note 该函数非线程安全，需要调用者自行保护数据安全
 */
static CanErrorCode_e _can_get_free_msg_list(CanMsgFifo_t Fifo, CanMsgList_t* MsgList)
{
    CanErrorCode_e ret = CAN_ERR_NONE;

    // 如果空闲链表非空，则取出一个链表项
    if (!list_empty(&Fifo->freeList))
    {
        *MsgList = list_first_entry(&Fifo->freeList, CanMsgList_s, fifoListNode);
        list_del(&(*MsgList)->fifoListNode); // 将该节点从空闲链表中删除
        Fifo->freeCount--;
    }
    // 如果空闲链表空，则代表链表已满。
    // 若开启了覆盖模式，则从已用链表中取出一个数据，相当于覆盖掉一个数据
    // 若未开启覆盖模式，则返回FIFO溢出错误
    else if (!list_empty(&Fifo->usedList) && Fifo->isOverwrite)
    {
        ret      = CAN_ERR_SOFT_FIFO_OVERFLOW;
        *MsgList = list_first_entry(&Fifo->usedList, CanMsgList_s, fifoListNode);
        list_del(&(*MsgList)->fifoListNode); // 将该节点从已用链表中删除
    }
    else if (!Fifo->isOverwrite) // 若未开启覆盖模式，则返回FIFO溢出错误
    {
        *MsgList = NULL;
        ret      = CAN_ERR_SOFT_FIFO_OVERFLOW;
    }
    // 按照道理来讲，空闲和已用链表的链表项数量和，始终为恒定的正整数
    // 出现两表皆空的情况，只能是缓存区未初始化
    else
    {
        while (1) {}; // TODO: ASSERT "Receive message buffer is not initialized"
    }
    return ret;
}

/**
 * @brief 将CAN消息链表项添加回CAN接收FIFO的空闲链表中
 * @param RxFifo CAN接收FIFO
 * @param MsgList CAN消息链表项
 * @retval 无
 *
 * @note 该函数非线程安全，需要调用者自行保护数据安全
 */
static inline void _can_add_free_msg_list(CanMsgFifo_t Fifo, CanMsgList_t MsgList)
{
    list_add_tail(&MsgList->fifoListNode, &Fifo->freeList); // 将该节点添加到空闲链表中
    Fifo->freeCount++;
}

/**
 * @brief 将CAN消息链表项添加回CAN发送FIFO的已用链表中
 * @param Fifo CANFIFO
 * @param MsgList CAN消息链表项
 *
 * @note 该函数非线程安全，需要调用者自行保护数据安全
 */
static inline void _can_add_used_msg_list(CanMsgFifo_t Fifo, CanMsgList_t MsgList)
{
    list_add_tail(&MsgList->fifoListNode, &Fifo->usedList); // 将该节点添加到已用链表中
}

/**
 * @brief 从CAN接收FIFO中获取一个空闲链表项
 *
 * @param RxHandler CAN接收句柄
 * @param MsgList CAN消息链表项指针
 * @return CanErrorCode_e 错误码
 * @retval 1. CAN_ERR_NONE          成功
 * @retval 2. CAN_ERR_SOFT_FIFO_OVERFLOW 接收FIFO溢出
 */
static CanErrorCode_e _canrx_get_free_msg_list(CanRxHandler_t RxHandler, CanMsgList_t* ppMsgList)
{
    uint32_t       intLevel;
    CanErrorCode_e ret = CAN_ERR_NONE;
    intLevel           = awlf_hw_disable_interrupt();
    ret                = _can_get_free_msg_list(&RxHandler->rxFifo, ppMsgList);
    if (*ppMsgList == NULL) // 若ppMsgList指向NULL，说明发生了某种错误，直接返回结果
    {
        awlf_hw_restore_interrupt(intLevel);
        return ret;
    }
    // 如果该链表项有匹配的滤波器，则将其从滤波器的匹配链表中删除
    if ((*ppMsgList)->owner != NULL)
    {
        list_del(&(*ppMsgList)->matchedListNode);
        // 如果是FIFO溢出，说明取得的是还来不及读取的数据，所以其原本匹配滤波器的消息数量需要减一
        if (ret == CAN_ERR_SOFT_FIFO_OVERFLOW)
            ((CanFilter_t)(*ppMsgList)->owner)->msgCount--;
        (*ppMsgList)->owner = NULL;
    }
    awlf_hw_restore_interrupt(intLevel);
    return ret;
}

/**
 * @brief 从CAN发送FIFO中获取一个空闲链表项
 * @param TxHandler CAN发送句柄
 * @param MsgList CAN消息链表项指针
 * @return CanErrorCode_e 错误码
 * @retval 1. CAN_ERR_NONE          成功
 * @retval 2. CAN_ERR_SOFT_FIFO_OVERFLOW 发送FIFO溢出
 *
 */
static CanErrorCode_e _cantx_get_free_msg_list(CanTxHandler_t TxHandler, CanMsgList_t* ppMsgList)
{
    uint32_t       intLevel;
    CanErrorCode_e ret = CAN_ERR_NONE;
    intLevel           = awlf_hw_disable_interrupt();
    ret                = _can_get_free_msg_list(&TxHandler->txFifo, ppMsgList);
    if (*ppMsgList == NULL) // 若ppMsgList指向NULL，说明发生了某种错误，直接返回结果
    {
        awlf_hw_restore_interrupt(intLevel);
        return ret;
    }
    while ((*ppMsgList)->owner != NULL && !TxHandler->txFifo.isOverwrite) {
    }; // TODO: 非覆写模式下不可能出现取到正在发送的报文的情况
    // 如果该链表项是正在发送的报文(覆写模式下会触发)，则将其从发送邮箱的匹配链表中删除
    // TODO: 这个逻辑可能导致正在发送或等待重传的报文被错误地从匹配链表中删除
    if ((*ppMsgList)->owner != NULL)
    {
        list_del(&(*ppMsgList)->matchedListNode);
        // 如果是FIFO溢出，说明取得的是还来不及发送的数据，所以其原本匹配发送邮箱的消息需要中断发送
        if (ret == CAN_ERR_SOFT_FIFO_OVERFLOW)
            ((CanMailbox_t)(*ppMsgList)->owner)->isBusy = 0;
        (*ppMsgList)->owner = NULL;
    }
    // TODO: 未来拓展TxHandler数据结构时，此处可能需要做额外操作
    awlf_hw_restore_interrupt(intLevel);
    return ret;
}

static inline void _canrx_add_free_msg_list(CanRxHandler_t RxHandler, CanMsgList_t MsgList)
{
    uint32_t intLevel;
    intLevel = awlf_hw_disable_interrupt();
    _can_add_free_msg_list(&RxHandler->rxFifo, MsgList); // 将该节点添加到空闲链表中
    awlf_hw_restore_interrupt(intLevel);
}

static inline void _cantx_add_free_msg_list(CanTxHandler_t TxHandler, CanMailbox_t mailbox)
{
    CanMsgList_t MsgList;
    uint32_t     intLevel;

    intLevel = awlf_hw_disable_interrupt();
    MsgList  = mailbox->pMsgList;
    list_del(&MsgList->fifoListNode); // 从已用链表中删除
    list_del(&MsgList->matchedListNode);
    mailbox->isBusy   = 0;
    mailbox->pMsgList = NULL;
    MsgList->owner    = NULL;
    list_add_tail(&mailbox->list, &TxHandler->mailboxList);
    _can_add_free_msg_list(&TxHandler->txFifo, MsgList); // 将该节点添加到空闲链表中
    awlf_hw_restore_interrupt(intLevel);
}

/**
 * @brief 将填充好数据的CAN消息链表（已用项）添加回CAN接收FIFO和滤波器匹配链表中
 * @param RxHandler CAN接收句柄
 * @param filterBank 接收过滤器编号
 * @param msgList CAN消息链表项
 * @retval 无
 * @note Filter合法性由函数调用者保证，本函数不做检查
 */
static inline void _canrx_add_used_msg_list(CanRxHandler_t RxHandler, CanFilter_t Filter, CanMsgList_t MsgList)
{
    uint32_t intLevel;
    intLevel = awlf_hw_disable_interrupt();
    list_add_tail(&MsgList->matchedListNode, &Filter->msgMatchedList); // 将该节点添加到滤波器匹配链表中
    MsgList->owner = Filter;                                           // 记录该消息链表项所属的滤波器
    Filter->msgCount++;                                                // 增加滤波器匹配消息数量
    _can_add_used_msg_list(&RxHandler->rxFifo, MsgList);               // 将该节点添加到已用链表中
    awlf_hw_restore_interrupt(intLevel);
}

static inline void cantx_add_used_msg_list(CanTxHandler_t TxHandler, CanMsgList_t pMsgList)
{
    uint32_t intLevel;
    intLevel = awlf_hw_disable_interrupt();
    _can_add_used_msg_list(&TxHandler->txFifo, pMsgList); // 将该节点添加到已用链表中
    awlf_hw_restore_interrupt(intLevel);
}

/**
 * @brief 从CAN接收FIFO中获取一个CAN消息链表项，同时将其从已用链表和滤波器匹配链表中删除
 * @param RxHandler CAN接收句柄
 * @param Filter 接收滤波器
 * @param pRet 错误码指针
 * @return CAN消息链表项指针
 * @note filterBank合法性由函数调用者保证，本函数不做检查
 */
static CanMsgList_t canrx_get_used_msg_list(CanRxHandler_t RxHandler, CanFilter_t Filter)
{
    uint32_t     intLevel;
    CanMsgList_t MsgList;
    intLevel = awlf_hw_disable_interrupt();
    // 如果指定了滤波器，且该滤波器有匹配的消息链表项，则取出第一个匹配项
    if (Filter != NULL)
    {
        MsgList = list_first_entry(&Filter->msgMatchedList, CanMsgList_s, matchedListNode);
        Filter->msgCount--;
        MsgList->owner = NULL;
    }
    // 如果没有指定滤波器，则从已用链表中取出一个链表项
    else
    {
        MsgList = list_first_entry(&RxHandler->rxFifo.usedList, CanMsgList_s, fifoListNode);
    }
    list_del(&MsgList->fifoListNode);    // 将该节点从已用链表中删除
    list_del(&MsgList->matchedListNode); // 将该节点从滤波器匹配链表中删除
    awlf_hw_restore_interrupt(intLevel);
    return MsgList;
}

static inline CanMsgList_t cantx_get_used_msg_list(CanTxHandler_t TxHandler)
{
    uint32_t     intLevel;
    CanMsgList_t MsgList;
    intLevel = awlf_hw_disable_interrupt();
    MsgList  = list_first_entry(&TxHandler->txFifo.usedList, CanMsgList_s, fifoListNode);
    list_del(&MsgList->fifoListNode); // 将该节点从已用链表中删除
    awlf_hw_restore_interrupt(intLevel);
    return MsgList;
}

/*
 * @brief 添加接收消息到CAN接收FIFO和滤波器匹配链表中
 * @param Can CAN句柄
 * @param userRxMsg CAN用户接收消息指针
 * @return void
 */
static void canrx_msg_put(HalCanHandler_t Can, CanMsgList_t HwRxMsgList)
{
    CanRxHandler_t RxHandler;
    CanFilter_t    Filter;
    CanUserMsg_t   HwRxMsg;
    int32_t        filterBank;
    while (!HwRxMsgList) {}; // TODO: assert
    HwRxMsg = &HwRxMsgList->userMsg;
    // 参数检查与初始化
    // TODO: ASSERT "User message is NULL"
    while (!HwRxMsg->userBuf) {}; // TODO: assert
    filterBank = HwRxMsg->bank;
    // TODO: ASSERT "Filter bank %d is out of range"
    while (IS_CAN_FILTER_INVALID(Can, filterBank)) {};

    Filter    = NULL;
    RxHandler = &Can->rxHandler;
    Filter    = &RxHandler->filterTable[filterBank];

    // 将消息链表添加回已用链表和滤波器匹配链表中
    _canrx_add_used_msg_list(RxHandler, Filter, HwRxMsgList);

    if (Filter != NULL && Filter->cfg.rx_callback != NULL)
    {
        Filter->cfg.rx_callback(&Can->parent, Filter->cfg.param, filterBank, Filter->msgCount);
    }
    else
    {
        // 由于当前CAN总线必须要设置一个滤波器，而设置滤波器必须要设置rx_callback，所以大概率不会跳到这里
        // 而且暂时也想不到device模型的read_cb在CAN中有什么价值，所以暂时不处理
        while (1) {}; // TODO: ASSERT "CAN filter bank %d RX callback is NULL"
        // // size_t msgCount = Can->cfg.rxMsgListBufSize - RxHandler->rxFifo.freeCount;
        // // device_read_cb(&Can->parent, msgCount);
    }
}

/**
 * @brief 从CAN接收FIFO中获取一个CAN消息，非阻塞
 * @param Can CAN句柄
 * @param pUserRxMsg CAN用户接收消息指针
 * @retval  1. CAN_ERR_NONE           成功
 * @retval  2. CANRX_ERR_FILTER_EMPTY，滤波器为空，则返回 NULL
 * @retval  4. CANRX_ERR_SOFT_FIFO_EMPTY   接收FIFO为空
 */
static CanErrorCode_e canrx_msg_get_noblock(CanRxHandler_t rxHandler, CanUserMsg_t pUserRxMsg)
{
    CanFilter_t  Filter;
    size_t       filterBank;
    uint32_t     intLevel;
    CanMsgList_t MsgList;
    filterBank = pUserRxMsg->bank;
    MsgList    = NULL;
    Filter     = NULL;

    intLevel = awlf_hw_disable_interrupt();
    Filter   = &rxHandler->filterTable[filterBank];
    if (Filter->msgCount == 0) // 非阻塞，直接退出
    {
        // TODO: ASSERT "Filter bank %d is empty"
        awlf_hw_restore_interrupt(intLevel);
        return CAN_ERR_FIFO_EMPTY;
    }
    else if (list_empty(&rxHandler->rxFifo.usedList))
    {
        // TODO: ASSERT "Soft FIFO is empty"
        awlf_hw_restore_interrupt(intLevel);
        return CAN_ERR_FIFO_EMPTY;
    }
    awlf_hw_restore_interrupt(intLevel);
    // 开始获取报文
    // 从rxFifo中取出一个已用链表项
    MsgList = canrx_get_used_msg_list(rxHandler, Filter);
    // 从这里之后，MsgList已经从rxFifo和Filter中被取出，
    // 此时除了当前操作之外，已经没有任何代码能访问这个链表项，所以针对MsgList的操作无需考虑竞态

    // 将容器中的报文拷贝到用户接收消息指针中
    _can_container_copy_to_usermsg(MsgList, pUserRxMsg);
    // 将该消息链表项添加回空闲链表中
    _can_add_free_msg_list(&rxHandler->rxFifo, MsgList);
    return CAN_ERR_NONE;
}

/**
 * @brief CAN 发送调度器
 * @param Can CAN句柄
 * @note 核心逻辑：从待发送 FIFO 中取数据 -> 填入空闲的硬件邮箱 -> 触发发送
 * @note 该函数可以在用户线程中触发，也可以在 ISR 中调用
 */
static void _can_tx_scheduler(HalCanHandler_t Can)
{
    CanTxHandler_t TxHandler = &Can->txHandler;
    CanMailbox_t   mailbox;
    CanMsgList_t   pMsgList;
    uint32_t       intLevel;
    int            i;

    // 进入临界区，保护 FIFO 和 Mailbox 状态
    intLevel = awlf_hw_disable_interrupt();

    if (list_empty(&TxHandler->txFifo.usedList))
    {
        // 没有数据要发了，退出循环
        awlf_hw_restore_interrupt(intLevel);
        return;
    }
    // TODO: 完善mailbox机制，使我们能够主动调度mailbox
    // TODO: 当邮箱数量多了起来之后可能会在中断中循环太多次，这是一个隐患，不过考虑到CAN总线目前的负载能力
    // 各家CAN IP都不会有太多的邮箱，所以暂时不需要考虑。即便未来要改，由于架构设计得当，改动也会被限制在这个函数中
    while (!list_empty(&TxHandler->mailboxList))
    {
        pMsgList      = list_first_entry(&TxHandler->txFifo.usedList, CanMsgList_s, fifoListNode);
        AwlfRet_e ret = Can->hwInterface->send_msg_mailbox(Can, &pMsgList->userMsg);
        if (ret == AWLF_OK)
        {
            // 发送成功
            while (IS_CAN_MAILBOX_INVALID(Can, pMsgList->userMsg.bank)) {}; // TODO: assert
            mailbox = &TxHandler->pMailboxes[pMsgList->userMsg.bank];
            // 从等待队列(usedList)中移除消息
            list_del(&pMsgList->fifoListNode);
            // 从可用邮箱中移除邮箱
            list_del(&mailbox->list);
            // 标记邮箱忙
            mailbox->isBusy   = 1;
            mailbox->pMsgList = pMsgList;
            pMsgList->owner   = mailbox;
        }
        else
        {
            // TODO: 进入这里通常说明进入 BUS_OFF状态 或是其他未知原因
            // 这里简单处理，直接退出循环
            // TODO: LOG "CAN status: %d，发送失败，bank: %d，msgID: 0x%08x"
            if (ret == AWLF_ERR_OVERFLOW)
                Can->statusManager.errCounter.txMailboxFullCnt++;
            break;
        }
        if (list_empty(&TxHandler->txFifo.usedList))
            break;
    }
    awlf_hw_restore_interrupt(intLevel);
}

/**
 * @brief 向CAN发送FIFO中添加一个CAN消息，非阻塞
 *
 * @param Can CAN句柄
 * @param pUserTxMsgBuf CAN用户发送消息数组指针
 * @param msgNum 消息数量
 * @return size_t 实际发送的消息数量
 */
static size_t cantx_msg_put_nonblock(HalCanHandler_t Can, CanUserMsg_t pUserTxMsgBuf, size_t msgNum)
{
    uint32_t     intLevel;
    size_t       msgCounter = 0;
    CanMsgList_t pMsgList   = NULL;
    CanMsgFifo_t txFifo     = &Can->txHandler.txFifo;
    CanMailbox_t mailbox    = NULL;
    for (msgCounter = 0; msgCounter < msgNum; msgCounter++)
    {
        // 获取一个空闲消息链表项，用于存储信息
        if (_cantx_get_free_msg_list(&Can->txHandler, &pMsgList) == CAN_ERR_SOFT_FIFO_OVERFLOW)
        {
            if (!Can->txHandler.txFifo.isOverwrite)
            {
                intLevel = awlf_hw_disable_interrupt();
                Can->statusManager.errCounter.txSoftOverFlowCnt += (msgNum - msgCounter);
                Can->statusManager.errCounter.txFailCnt += (msgNum - msgCounter); // 被覆盖的消息定义为发送失败的消息
                awlf_hw_restore_interrupt(intLevel);
                break; // 跳出循环，结束写入
            }
            else // 覆写
            {
                awlf_hw_restore_interrupt(intLevel);
                Can->statusManager.errCounter.txFailCnt++; // 未被发送的消息定义为发送失败的消息
                Can->statusManager.errCounter.txSoftOverFlowCnt++;
                awlf_hw_restore_interrupt(intLevel);
            }
        }
        // 从这里之后，pMsgList已经从Fifo中被取出
        // 此时除了当前操作之外，已经没有任何代码能访问这个链表项，所以针对pMsgList的操作无需考虑竞态

        // 填充用户消息指针
        pMsgList->userMsg = pUserTxMsgBuf[msgCounter];
        // 填充CAN消息容器
        memcpy((void*)pMsgList->container, (void*)pUserTxMsgBuf[msgCounter].userBuf, pUserTxMsgBuf[msgCounter].dsc.dataLen);

        intLevel = awlf_hw_disable_interrupt();
        _can_add_used_msg_list(&Can->txHandler.txFifo, pMsgList);
        awlf_hw_restore_interrupt(intLevel);
    }
    // 在中断上下文中不需要主动调度
    if (!xPortIsInsideInterrupt())
        _can_tx_scheduler(Can);
    return msgCounter;
}

/**
 * @brief CAN软件重传机制
 *
 * @param Can CAN设备指针
 * @param mailbox 邮箱指针
 */
#include "core/awlf_cpu.h"
__dbg_param_def(float, dbgCanTxTime) = 0.0f;
static void cantx_soft_retransmit(HalCanHandler_t Can, uint32_t mailboxBank)
{
    // 因为尝试的所有邮箱和消息节点都会被从各自的链表中摘除，所以触发仲裁/发送错误的邮箱和节点都是外界不可访问的，不需要考虑竞态（仅针对邮箱和该消息链表而言）
    CanMailbox_t mailbox  = &Can->txHandler.pMailboxes[mailboxBank];
    CanMsgList_t pMsgList = mailbox->pMsgList;
    while (!pMsgList) {}; // TODO： assert
    pMsgList->owner = NULL;
    uint32_t intLevel;
    intLevel = awlf_hw_disable_interrupt();
    list_add(&pMsgList->fifoListNode, &Can->txHandler.txFifo.usedList); // 头插，确保先发送的消息先被发送
    awlf_hw_restore_interrupt(intLevel);
    mailbox->isBusy   = 0;
    mailbox->pMsgList = NULL;
    list_add_tail(&mailbox->list, &Can->txHandler.mailboxList);
    dbgCanTxTime = awlf_cpu_get_time_s();
    _can_tx_scheduler(Can);
}

/*
 * @brief 初始化CAN设备
 * @param Dev CAN设备指针
 * @retval  1. AWLF_ERROR_PARAM 参数错误
 *          2. AWLF_ERROR_HW 硬件错误
 *          3. AWLF_ERROR_NONE 成功
 */
static AwlfRet_e can_init(Device_t Dev)
{
    AwlfRet_e ret;
    if (!Dev)
        return AWLF_ERROR_PARAM;
    HalCanHandler_t Can = (HalCanHandler_t)Dev;
    if (!Can->adapterInterface || !Can->hwInterface)
        return AWLF_ERROR_PARAM;
    _can_rxhandler_deinit(&Can->rxHandler);
    _can_txhandler_deinit(&Can->txHandler);
    ret = Can->hwInterface->configure(Can, &Can->cfg);
    return ret;
}

static AwlfRet_e can_open(Device_t Dev, uint32_t oparam)
{
    AwlfRet_e ret;
    uint32_t  iotype;
    if (!Dev)
        return AWLF_ERROR_PARAM;
    HalCanHandler_t Can = (HalCanHandler_t)Dev;

    iotype = oparam & DEVICE_O_RXTYPE_MASK;

    ret = _can_rxhandler_init(Can, iotype, Can->cfg.filterNum, Can->cfg.rxMsgListBufSize);
    if (ret != AWLF_OK)
        return ret;

    iotype = oparam & DEVICE_O_TXTYPE_MASK;
    ret    = _can_txhandler_init(Can, iotype, Can->cfg.mailboxNum, Can->cfg.txMsgListBufSize);
    if (ret != AWLF_OK)
    {
        // TODO: LOG ERR
        _can_rxhandler_deinit(&Can->rxHandler);
        return ret;
    }
    ret = _can_status_manager_init(Can);
    if (ret != AWLF_OK)
    {
        // TODO: LOG ERR
        _can_rxhandler_deinit(&Can->rxHandler);
        _can_txhandler_deinit(&Can->txHandler);
        return ret;
    }
    return ret;
}

static size_t can_write(Device_t Dev, void* pos, void* data, size_t len)
{
    size_t          msgNum = 0;
    HalCanHandler_t Can    = (HalCanHandler_t)Dev;
    // 目前只支持非阻塞发送策略
    msgNum = cantx_msg_put_nonblock(Can, (CanUserMsg_t)data, len);
    return msgNum;
}

/**
 * @brief 从CAN接收FIFO中读取多个CAN消息
 *
 * @param Dev CAN设备指针
 * @param pos CAN 不需要该参数，保持NULL即可
 * @param buf 接收消息缓冲区指针
 * @param len 接收消息缓冲区长度
 * @return size_t 实际读取的消息数量
 */
static size_t can_read(Device_t Dev, void* pos, void* buf, size_t len)
{
    size_t          cnt           = 0;
    HalCanHandler_t Can           = (HalCanHandler_t)Dev;
    CanRxHandler_t  rxHandler     = &Can->rxHandler;
    CanErrorCode_e  ret           = CAN_ERR_NONE;
    CanUserMsg_t    pUserRxMsgBuf = (CanUserMsg_t)buf;

    for (cnt = 0; cnt < len; cnt++)
    {
        while (!pUserRxMsgBuf[cnt].userBuf) {}; // TODO: assert
        int32_t filterBank = pUserRxMsgBuf[cnt].bank;
        while (IS_CAN_FILTER_INVALID(Can, filterBank)) {};            // TODO: ASSERT "Filter bank %d is INVALID"
        while (rxHandler->filterTable[filterBank].isActived == 0) {}; // TODO: ASSERT "Filter bank %d is not active"
        ret = canrx_msg_get_noblock(rxHandler, &pUserRxMsgBuf[cnt]);
        if (ret != CAN_ERR_NONE)
            break;
    }
    if (ret != CAN_ERR_NONE)
    {
        // TODO: LOG ERR
        device_err_cb(&Can->parent, ret, cnt);
    }
    return cnt;
}

/**
 * @brief 控制CAN设备
 * @param Dev CAN设备指针
 * @param cmd 命令 @ref CAN_CMD_DEF
 * @param args 参数指针
 * @retval  1. AWLF_ERROR_PARAM 参数错误
 *          2. AWLF_ERROR_NONE 成功
 */
static AwlfRet_e can_ctrl(Device_t Dev, size_t cmd, void* args)
{
    AwlfRet_e ret = AWLF_OK;
    if (!Dev)
        return AWLF_ERROR_PARAM;
    HalCanHandler_t Can = (HalCanHandler_t)Dev;
    switch (cmd)
    {
        case CAN_CMD_SET_IOTYPE: ret = Can->hwInterface->control(Can, CAN_CMD_SET_IOTYPE, args); break;

        case CAN_CMD_CLR_IOTYPE: ret = Can->hwInterface->control(Can, CAN_CMD_CLR_IOTYPE, NULL); break;

        // 学习者需注意这里的程序设计，检查参数合法性，并且操作失败后也不会产生副作用（即如果配置失败，CAN设备的状态或数据不会改变），这在架构设计中是必要的
        case CAN_CMD_CFG:
            if (!args)
            {
                ret = AWLF_ERROR_PARAM;
                break;
            }
            ret = Can->hwInterface->control(Can, CAN_CMD_CFG, args);
            if (ret == AWLF_OK)
                Can->cfg = *(CanCfg_t)args;
            break;

        // 学习者需注意这里的程序设计，检查参数合法性，并且操作失败后也不会产生副作用（即如果配置失败，CAN设备的状态或数据不会改变），这在架构设计中是必要的
        case CAN_CMD_SET_FILTER:
        {
            CanFilter_t    filter;
            CanFilterCfg_t cfg = (CanFilterCfg_t)args;
            if (IS_CAN_FILTER_INVALID(Can, cfg->bank) || cfg->rx_callback == NULL)
            {
                ret = AWLF_ERROR_PARAM;
                break;
            }
            // 如果滤波器已经被激活，则返回错误
            filter = &Can->rxHandler.filterTable[cfg->bank];
            if (filter->isActived)
            {
                ret = AWLF_ERROR_BUSY;
                break;
            }
            ret = Can->hwInterface->control(Can, CAN_CMD_SET_FILTER, (void*)cfg);
            if (ret != AWLF_OK)
                break;
            Can->rxHandler.filterTable[cfg->bank].cfg = *cfg; // 确保操作都成功后才能赋值
            filter->isActived                         = 1;
            filter->msgCount                          = 0;
        }
        break;

        case CAN_CMD_START: ret = Can->hwInterface->control(Can, CAN_CMD_START, NULL); break;

        case CAN_CMD_CLOSE: ret = Can->hwInterface->control(Can, CAN_CMD_CLOSE, NULL); break;
        default: ret = Can->hwInterface->control(Can, cmd, args); break;
    }
    return ret;
}

static AwlfRet_e can_close(Device_t Dev)
{
    AwlfRet_e ret;
    if (!Dev)
        return AWLF_ERROR_PARAM;
    HalCanHandler_t Can = (HalCanHandler_t)Dev;
    if (!Can->adapterInterface || !Can->hwInterface)
        return AWLF_ERROR_PARAM;
    Can->hwInterface->control(Can, CAN_CMD_CLR_IOTYPE, NULL);
    Can->hwInterface->control(Can, CAN_CMD_CLOSE, NULL);
    _can_rxhandler_deinit(&Can->rxHandler);
    _can_txhandler_deinit(&Can->txHandler);
    _can_status_manager_deinit(Can);
    return AWLF_OK;
}

/*
 * @brief CAN错误中断处理函数
 * @param Can CAN设备指针
 * @param errorEvent 错误事件
 * @retval 无
 * @note 该函数主要更新CAN设备的工作状态，更具体的处理逻辑将放在CAN设备的状态管理线程当中。
 */
void can_error_isr(HalCanHandler_t Can, uint32_t errEvent, size_t param)
{
    switch (errEvent)
    {
        case CAN_ERR_EVENT_TX_FAIL:
            Can->statusManager.errCounter.txFailCnt++;
            while (IS_CAN_MAILBOX_INVALID(Can, param)) {}; // TODO： assert
            cantx_soft_retransmit(Can, param);
            break;
        case CAN_ERR_EVENT_ARBITRATION_FAIL:
        {

            // TODO: LOG
            Can->statusManager.errCounter.txArbitrationFailCnt++;
            while (IS_CAN_MAILBOX_INVALID(Can, param)) {}; // TODO： assert
            cantx_soft_retransmit(Can, param);
        }
        break;
        case CAN_ERR_EVENT_BUS_STATUS: // TODO: 处理总线错误
        {
            size_t canTxErrCnt = Can->statusManager.errCounter.txErrCnt;
            size_t canRxErrCnt = Can->statusManager.errCounter.rxErrCnt;
            if (canRxErrCnt < 127 && canTxErrCnt < 127)
                Can->statusManager.nodeErrStatus = CAN_NODE_STATUS_ACTIVE;
            else if (canRxErrCnt > 127 || canTxErrCnt > 127)
                Can->statusManager.nodeErrStatus = CAN_NODE_STATUS_PASSIVE;
            else if (canTxErrCnt > 255)
                Can->statusManager.nodeErrStatus = CAN_NODE_STATUS_BUSOFF;
            device_err_cb(&Can->parent, CAN_ERR_EVENT_BUS_STATUS, Can->statusManager.nodeErrStatus);
        }
        break;
    }
}

void hal_can_isr(HalCanHandler_t Can, CanIsrEvent_e event, size_t param)
{
    switch (event)
    {
        case CAN_ISR_EVENT_INT_RX_DONE: // 接收完成中断，param是接收消息的FIFO索引
        {
            AwlfRet_e    ret;
            CanMsgList_t MsgList = NULL;
            if (_canrx_get_free_msg_list(&Can->rxHandler, &MsgList) == CAN_ERR_SOFT_FIFO_OVERFLOW)
            {
                Can->statusManager.errCounter.rxSoftOverFlowCnt++; // 接收软件FIFO溢出计数器增加
                if (!Can->rxHandler.rxFifo.isOverwrite)            // 非覆写策略直接退出
                {
                    Can->hwInterface->recv_msg(Can, NULL, param); // 丢弃当前帧（清中断）
                    break;
                }
            }
            ret = Can->hwInterface->recv_msg(Can, &MsgList->userMsg, param);
            if (ret == AWLF_OK)
            {
                canrx_msg_put(Can, MsgList);
                Can->statusManager.errCounter.rxMsgCount++; // 接收消息计数器增加
            }
            else
            {
                Can->statusManager.errCounter.rxFailCnt++; // 接收错误计数器增加
            }
        }
        break;
        case CAN_ISR_EVENT_INT_TX_DONE:                             // 发送完成中断，param是邮箱编号
            while (IS_CAN_MAILBOX_INVALID(Can, (int32_t)param)) {}; // TODO: assert
            CanMailbox_t mailbox = &Can->txHandler.pMailboxes[param];

            // 资源回收
            uint32_t intLevel = awlf_hw_disable_interrupt();
            _cantx_add_free_msg_list(&Can->txHandler, mailbox);
            Can->statusManager.errCounter.txMsgCount++; // 发送消息计数器增加
            device_write_cb(&Can->parent, param);
            _can_tx_scheduler(Can);
            awlf_hw_restore_interrupt(intLevel);
            break;
        default:
        {
            volatile uint32_t debug = 0;
        }
        break;
    }
}

static DevInterface_s CanDevInterface = {
    .init    = can_init,
    .open    = can_open,
    .write   = can_write,
    .read    = can_read,
    .control = can_ctrl,
    .close   = can_close,
};

/**
 * @brief CAN设备注册（兼容经典CAN和CANFd）
 * @param can CAN设备句柄
 * @param name 设备名称
 * @param handle 硬件句柄
 * @param regparams 注册参数 @ref CAN_REG_DEF
 * @return AwlfRet_e 注册结果
 * @retval AWLF_OK 成功
 * @retval AWLF_ERROR_PARAM 参数错误
 * @retval AWLF_ERR_CONFLICT 设备名称冲突
 */
AwlfRet_e hal_can_register(HalCanHandler_t can, char* name, void* handle, uint32_t regparams)
{
    AwlfRet_e ret;
    if (!can || !name || !can->adapterInterface)
        return AWLF_ERROR_PARAM;
    ret = device_register(&can->parent, name, regparams);
    if (ret != AWLF_OK)
        return ret;

    can->parent.handle    = handle;
    can->parent.interface = &CanDevInterface;
    can->cfg              = CAN_DEFUALT_CFG;
    return ret;
}
