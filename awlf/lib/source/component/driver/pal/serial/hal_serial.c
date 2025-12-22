#include "component/driver/pal/pal_serial_dev.h"
#include "aw_core/awlf_interrupt.h"
/*************************** PRIVATE MACRO ********************************************************/
#define SERIAL_IS_O_BLCK_TX(dev)  (device_get_oparams(dev) & SERIAL_O_BLCK_TX)
#define SERIAL_IS_O_NBLCK_TX(dev) (device_get_oparams(dev) & SERIAL_O_NBLCK_TX)
#define SERIAL_IS_O_BLCK_RX(dev)  (device_get_oparams(dev) & SERIAL_O_BLCK_RX)
#define SERIAL_IS_O_NBLCK_RX(dev) (device_get_oparams(dev) & SERIAL_O_NBLCK_RX)
/************************** PRIVATE FUNCTION ********************************************************/
static AwlfRet_e serial_ctrl(Device_t dev, size_t cmd, void* arg);
/* 无FIFO情况下的 TX 函数*/
static size_t _serial_tx_poll(Device_t dev, void* data, size_t len)
{
    size_t      length;
    HalSerial_t serial = (HalSerial_t)dev;
    uint8_t*    _data  = (uint8_t*)data;
    if (device_check_status(dev, DEV_STATUS_BUSY_TX))
    {
        // TODO: log txbusy
        return 0;
    }
    device_set_status(dev, DEV_STATUS_BUSY_TX);
    for (length = 0; length < len; length++)
    {
        if (serial->interface->putByte(serial, _data[length]) != AWLF_OK)
            break;
    }
    // if(length < len)
    // TODO: LOG 发送超时，打印实际发送字节
    device_clr_status(dev, DEV_STATUS_BUSY_TX);
    return length;
}

static size_t _serial_rx_poll(Device_t dev, uint8_t* buf, size_t len)
{
    size_t      length;
    AwlfRet_e   ret    = AWLF_ERROR;
    HalSerial_t serial = (HalSerial_t)dev;
    if (device_check_status(dev, DEV_STATUS_BUSY_RX))
    {
        // TODO: log txbusy
        return 0;
    }
    device_set_status(dev, DEV_STATUS_BUSY_RX);
    for (length = 0; length < len; length++)
    {
        ret = serial->interface->getByte(serial, (buf++));
        if (ret != AWLF_OK)
            break;
    }
    // if(length < len)
    // TODO: LOG 接收超时，打印实际接收字节
    device_clr_status(dev, DEV_STATUS_BUSY_RX);
    return length;
}

/**
 * @brief 串口阻塞发送 内部框架实现
 *
 * @param dev 串口设备
 * @param data 应用层数据包指针
 * @param len  应用层数据包长度
 * @return size_t 实际发送长度
 * @note   调用者需自行确保data的内存安全和数据完整性
 */
static size_t _serial_tx_block(Device_t dev, void* data, size_t len)
{
    HalSerial_t  serial;
    SerialFifo_t txFifo;
    uint32_t     primask;
    size_t       txlen;
    size_t       linear_space_len;
    void*        tx_linear_buf = 0; // 串口线性缓存区
    size_t       sended        = 0; // 已经发送的长度

    serial = (HalSerial_t)dev;
    txFifo = serial_get_txfifo(serial);

    // 理论上对于串口，对于阻塞发送来说不应该出现BUSY_TX的情况.
    // 因为同一个串口，同一时间仅允许被一个线程打开
    device_set_status(dev, DEV_STATUS_BUSY_TX);
    do
    {
        if (txFifo->status == SERIAL_FIFO_IDLE)
        {
            txlen            = ringbuf_in(&txFifo->rb, data + sended, len - sended);
            linear_space_len = ringbuf_get_item_linear_space(&txFifo->rb, &tx_linear_buf);
            txFifo->loadSize = linear_space_len;
            txFifo->status   = SERIAL_FIFO_BUSY;
            serial->interface->transmit(serial, tx_linear_buf, linear_space_len);
            completion_wait(&txFifo->cpt, WAIT_FOREVER);
            sended += txlen;
        }
    } while (sended < len);

    device_clr_status(dev, DEV_STATUS_BUSY_TX);
    return sended;
}

/* 串口非阻塞发送 */
static size_t _serial_tx_nonblock(Device_t dev, void* data, size_t len)
{
    HalSerial_t  serial;
    SerialFifo_t txFifo;
    uint32_t     primask;
    size_t       tx_len;
    size_t       sended = 0;
    void*        tx_ptr = NULL;

    serial = (HalSerial_t)dev;
    txFifo = serial_get_txfifo(serial);
    sended = ringbuf_in(&txFifo->rb, data, len);

    device_set_status(dev, DEV_STATUS_BUSY_TX);
    /* 发送数据 */
    if (txFifo->status == SERIAL_FIFO_IDLE && sended)
    {
        tx_len           = ringbuf_get_item_linear_space(&txFifo->rb, &tx_ptr);
        txFifo->status   = SERIAL_FIFO_BUSY;
        txFifo->loadSize = tx_len;
        if (tx_len > 0)
            serial->interface->transmit(serial, tx_ptr, tx_len);
    }
    // 非阻塞的serial_busy_tx状态在中断中清除
    return sended;
}

static size_t _serial_rx_block(Device_t dev, void* buf, size_t len)
{
    HalSerial_t  serial;
    SerialFifo_t rxFifo;
    uint32_t     primask;
    size_t       rd_remain;      // 剩余未读取的长度
    size_t       rd_len;         // 本轮读取的长度，真实反应硬件实际接收字节数，也就是考虑rxfifo可能溢出的情况
    size_t       rd_from_rb = 0; // 应用层通过read接口获得的总长度，也就是不包括接收过程中rxfifo可能溢出的情况

    serial = (HalSerial_t)dev;
    rxFifo = serial_get_rxfifo(serial);
    rd_len = ringbuf_len(&rxFifo->rb);

    /* rd_len < len */
    rd_remain = len - rd_len; // 减去已经有的数据长度才是剩余需要读取的长度
    rd_from_rb += ringbuf_out(&rxFifo->rb, buf, rd_len);
    do
    {
        primask          = awlf_hw_disable_interrupt();
        rxFifo->loadSize = (rd_remain > ringbuf_avail(&rxFifo->rb)) ? ringbuf_avail(&rxFifo->rb) : rd_remain;
        rd_len           = rxFifo->loadSize;
        rxFifo->status   = SERIAL_FIFO_BUSY; // FIFO开始等待数据
        awlf_hw_restore_interrupt(primask);
        completion_wait(&rxFifo->cpt, WAIT_FOREVER); // 等待数据到来
        rxFifo->status = SERIAL_FIFO_IDLE;           // FIFO结束数据等待

        rd_len = (ringbuf_len(&rxFifo->rb) < rd_len) ? ringbuf_len(&rxFifo->rb) : rd_len;
        rd_from_rb += ringbuf_out(&rxFifo->rb, buf + rd_from_rb,
                                  rd_len); // rd_total仅考虑通过API调用获得的数据，这是为了让应用层知道通过API实际读取的字节数

        if (rxFifo->loadSize < 0) // loadSize < 0 说明出现了FIFO溢出错误
        {
            primask = awlf_hw_disable_interrupt();
            rd_len -=
                rxFifo
                    ->loadSize; // 负负得正，实际上是算上了溢出的数据长度(框架层不关心应用层是否处理溢出，只是诚实地记录硬件实际接收过的字节数)
            rxFifo->loadSize =
                0; // 至于溢出的数据怎么办？那是API使用者要考虑的事情，要不把rb开大一点，要不就在错误回调中做特殊处理
            awlf_hw_restore_interrupt(primask);
        }
        /*
            loadSize包含rxfifo溢出的信息，没法确定究竟溢出了多少，因此rd_len可能会大于rd_remain
            当rd_len>=rd_remain时，rd_remain直接归零即可，
        */
        rd_remain = (rd_remain > rd_len) ? (rd_remain - rd_len) : 0;
    } while (rd_remain);
    return rd_from_rb;
}

static size_t _serial_rx_nonblock(Device_t dev, void* buf, size_t len)
{
    HalSerial_t  serial;
    SerialFifo_t rxFifo;
    size_t       rd_len;

    serial = (HalSerial_t)dev;
    rxFifo = serial_get_rxfifo(serial);

    if (ringbuf_len(&rxFifo->rb) >= len)
        rd_len = ringbuf_out(&rxFifo->rb, buf, len);
    else
        rd_len = 0;
    return rd_len;
}

static AwlfRet_e _serial_set_cfg(HalSerial_t serial, SerialCfg_t cfg)
{
    AwlfRet_e ret;

    while (!serial) {}; // TODO: assert
    while (!cfg) {}; // TODO: assert

    if (cfg->txBufSize != serial->cfg.txBufSize || cfg->rxBufSize != serial->cfg.rxBufSize)
        return AWLF_ERROR_PARAM;

    ret = serial->interface->configure(serial, cfg);
    if (ret == AWLF_OK)
        serial->cfg = *cfg;
    // TODO: LOG
    return ret;
}

// TODO: 待实现
static AwlfRet_e _serial_flush(HalSerial_t serial, uint32_t fifo_selector)
{
    switch (fifo_selector)
    {
        case SERIAL_TXFLUSH:
        {
        }
        break;
        case SERIAL_RXFLUSH:
        {
        }
        break;
        case SERIAL_TXRXFLUSH:
            _serial_flush(serial, SERIAL_TXFLUSH);
            _serial_flush(serial, SERIAL_RXFLUSH);
            break;
    }
    return AWLF_OK;
}

static AwlfRet_e _serial_tx_enable(Device_t dev)
{
    HalSerial_t  serial;
    SerialFifo_t txFifo;
    size_t       ctrl_arg = 0;

    serial = (HalSerial_t)dev;
    txFifo = serial_get_txfifo(serial);
    // 无缓存
    if (serial->cfg.txBufSize == 0)
    {
        // 串口无缓存只能使用poll方式
        return AWLF_OK;
    }

    /* 底层未填充 FIFO */
    if (!txFifo)
    {
        /*  当前框架下，除了poll之外所有发送方式都会配置rb，无论阻塞非阻塞。
            后续若是发现不合适的话可以再细分，目前暂不支持。
            但是无论如何，建议非阻塞发送都使用rb。
        */
        if (serial->cfg.txBufSize < SERIAL_MIN_TX_BUFSZ) // TODO: log
            serial->cfg.txBufSize = SERIAL_MIN_TX_BUFSZ;

        txFifo = (SerialFifo_t)pvPortMalloc(sizeof(SerialFifo_s));
        while (!txFifo) {}; // TODO: assert
        ringbuf_alloc(&txFifo->rb, sizeof(uint8_t), serial->cfg.txBufSize, pvPortMalloc);
        if (!txFifo->rb.buf)
        {
            vPortFree(txFifo);
            while (1) {}; // TODO: assert
        }
        serial->__priv.txFifo = txFifo;
    }

    if (device_get_oparams(dev) & SERIAL_O_BLCK_TX)
        completion_init(&txFifo->cpt, CompEvent_SerialTx);

    ctrl_arg = device_get_regparams(dev) & DEVICE_REG_TXTYPE_MASK;

    serial->interface->control(serial, SERIAL_CMD_SET_IOTPYE, (void*)ctrl_arg);

    return AWLF_OK;
}

static AwlfRet_e _serial_rx_enable(Device_t dev)
{
    HalSerial_t  serial;
    SerialFifo_t rxFifo;
    uint32_t     ctrl_arg;

    ctrl_arg = device_get_regparams(dev) & DEVICE_REG_RXTYPE_MASK;

    serial = (HalSerial_t)dev;
    rxFifo = serial_get_rxfifo(serial);

    if (serial->cfg.rxBufSize == 0)
    {
        // 无缓存，串口只能轮询读取
        return AWLF_OK;
    }

    if (!rxFifo)
    {
        if (serial->cfg.rxBufSize < SERIAL_MIN_RX_BUFSZ)
            serial->cfg.rxBufSize = SERIAL_MIN_RX_BUFSZ;

        rxFifo = (SerialFifo_t)pvPortMalloc(sizeof(SerialFifo_s));
        while (!rxFifo) {}; // TODO: assert
        ringbuf_alloc(&rxFifo->rb, sizeof(uint8_t), serial->cfg.rxBufSize, pvPortMalloc);
        if (!rxFifo->rb.buf)
        {
            vPortFree(rxFifo);
            while (1) {}; // TODO: assert
        }
        serial->__priv.rxFifo = rxFifo;
    }
    if (device_get_oparams(dev) & SERIAL_O_BLCK_RX)
        completion_init(&rxFifo->cpt, CompEvent_SerialRx);

    serial->interface->control(serial, SERIAL_CMD_SET_IOTPYE, (void*)ctrl_arg);
    return AWLF_OK;
}

/******************************************* DEVICE INTERFACE ********************************************/
static AwlfRet_e serial_init(Device_t dev)
{
    HalSerial_t serial = (HalSerial_t)dev;
    if (!serial || !serial->interface || !serial->interface->configure)
        return AWLF_ERROR_PARAM;
    serial->__priv.txFifo = NULL;
    serial->__priv.rxFifo = NULL;
    return serial->interface->configure(serial, &serial->cfg);
}

static AwlfRet_e serial_open(Device_t dev, uint32_t otype)
{
    HalSerial_t serial = (HalSerial_t)dev;
    if (!serial || !serial->interface || !serial->interface->control)
        return AWLF_ERROR_PARAM;

    /* 串口写方式 */
    dev->__priv.oparams |= (otype & SERIAL_O_BLCK_TX) ? SERIAL_O_BLCK_TX : (otype & SERIAL_O_NBLCK_TX) ? SERIAL_O_NBLCK_TX : 0U;
    /* 串口读方式 */
    dev->__priv.oparams |= (otype & SERIAL_O_BLCK_RX) ? SERIAL_O_BLCK_RX : (otype & SERIAL_O_NBLCK_RX) ? SERIAL_O_NBLCK_RX : 0U;

    _serial_tx_enable(dev);
    _serial_rx_enable(dev);
    return AWLF_OK;
}

static AwlfRet_e serial_ctrl(Device_t dev, size_t cmd, void* arg)
{
    AwlfRet_e   ret;
    HalSerial_t serial = (HalSerial_t)dev;
    if (!serial || !serial->interface || !serial->interface->control) // TODO: assert
        return AWLF_ERROR_PARAM;
    ret = AWLF_OK;
    switch (cmd)
    {
        case SERIAL_CMD_FLUSH:
        {
            uint32_t _arg = (uint32_t)arg;
            ret           = _serial_flush(serial, _arg);
        }
        break;

        case SERIAL_CMD_SET_CFG:
        {
            SerialCfg_t cfg;
            AwlfRet_e   ret;
            if (!arg)
            {
                ret = AWLF_ERROR_PARAM;
                break;
            }
            cfg = (SerialCfg_t)arg;
            ret = _serial_set_cfg(serial, cfg);
        }
        break;

        case SERIAL_CMD_SUSPEND:
            device_set_status(&serial->parent, DEV_STATUS_SUSPEND); // 挂起串口
            break;

        case SERIAL_CMD_RESUME:
            device_clr_status(&serial->parent, DEV_STATUS_SUSPEND); // 恢复串口
            break;

        default: ret = serial->interface->control(serial, cmd, arg);
    }

    return ret;
}

static size_t serial_write(Device_t dev, void* pos, void* data, size_t len)
{
    SerialFifo_t txfifo;
    HalSerial_t  serial;
    uint32_t     oparams;
    size_t       ret_len = 0;
    oparams              = device_get_oparams(dev);
    serial               = (HalSerial_t)dev;
    txfifo               = serial_get_txfifo(serial);

    if (serial->cfg.txBufSize == 0 || ((device_get_regparams(dev) & DEVICE_REG_TXTYPE_MASK) == DEVICE_REG_POLL_TX))
    {
        return _serial_tx_poll(dev, data, len);
    }
    else if (!txfifo || !txfifo->rb.buf)
    {
        // TODO: log no fifo
        return 0;
    }

    // 若之前的数据没有发送完，直接放入FIFO等待发送
    if (device_check_status(dev, DEV_STATUS_BUSY_TX))
    {
        ret_len = ringbuf_in(&txfifo->rb, data, len);
    }
    else
    {
        if ((oparams & SERIAL_O_NBLCK_TX) || xPortIsInsideInterrupt()) // 在中断中只能使用非阻塞发送
            ret_len = _serial_tx_nonblock(dev, data, len);
        else if (oparams & SERIAL_O_BLCK_TX)
            ret_len = _serial_tx_block(dev, data, len);
    }
    // 若是发送数据小于总长度，则认为TXFIFO溢出，触发回调函数
    // 从机制上来说，阻塞发送不会出现TXFIFO溢出的情况，非阻塞发送则可能由于FIFO不够而直接退出发送
    if (ret_len < len)
        device_err_cb(dev, ERR_SERIAL_TXFIFO_OVERFLOW, len - ret_len);
    return ret_len;
}

static size_t serial_read(Device_t dev, void* pos, void* buf, size_t len)
{
    HalSerial_t  serial;
    SerialFifo_t rxFifo;
    size_t       recv_len = 0;
    serial                = (HalSerial_t)dev;
    rxFifo                = serial_get_rxfifo(serial);
    if (serial->cfg.rxBufSize == 0 || ((device_get_regparams(dev) & DEVICE_REG_RXTYPE_MASK) == DEVICE_REG_POLL_RX))
    {
        return _serial_rx_poll(dev, (uint8_t*)buf, len);
    }
    while (!rxFifo || !rxFifo->rb.buf) {}; // TODO: assert

    if (ringbuf_len(&rxFifo->rb) >= len)
        recv_len = ringbuf_out(&rxFifo->rb, buf, len);
    else if (SERIAL_IS_O_NBLCK_RX(dev) || xPortIsInsideInterrupt()) // 在中断中只能使用非阻塞接收
        recv_len = _serial_rx_nonblock(dev, buf, len);
    else if (SERIAL_IS_O_BLCK_RX(dev))
        recv_len = _serial_rx_block(dev, buf, len);

    return recv_len;
}

static DevInterface_s serial_interface = {
    .init    = serial_init,
    .open    = serial_open,
    .close   = AWLF_NULL,   // 待完成
    .control = serial_ctrl, // 待完善
    .read    = serial_read,
    .write   = serial_write,
};

/******************************************* HW API ********************************************/
AwlfRet_e serial_register(HalSerial_t serial, char* name, void* handle, uint32_t regparams)
{
    if (!serial || !name)
        return AWLF_ERROR_PARAM;
    serial->parent.handle    = handle;
    serial->parent.interface = &serial_interface;
    serial->cfg              = SERIAL_DEFAULT_CFG;
    return device_register(&serial->parent, name, regparams | DEVICE_REG_STANDALONG | DEVICE_REG_RDWR);
}

// 串口中断服务函数，优化方向：采用类似Linux的方法，将中断分为上下部的异步处理，上部为硬中断，快进快出，负责记录状态；下部为工作队列，负责处理数据和业务逻辑
AwlfRet_e serial_hw_isr(HalSerial_t serial, SerialEvent_e event, void* arg, size_t arg_size)
{
    if (!serial || !serial->interface ||
        (!device_check_status(&serial->parent, DEV_STATUS_OPENED) || !device_check_status(&serial->parent, DEV_STATUS_INITED) ||
         device_check_status(&serial->parent, DEV_STATUS_SUSPEND))) // TODO: assert
        return AWLF_ERROR_PARAM;

    switch (event)
    {
        /* arg = 接收buffer，arg_size = 本次接收数据的大小 */
        case SERIAL_EVENT_INT_RXDONE: /* 中断接收完成 */
        case SERIAL_EVENT_DMA_RXDONE: /* DMA接收完成 */
        {
            SerialFifo_t rxFifo;
            size_t       rx_len;   // 本次接收实际写入rb的数据长度
            size_t       data_len; // rb数据总长度
            while (!arg) {}; // TODO: assert
            while (!arg_size) {}; // TODO: assert, arg_size必须被传入，以更新loadSize
            rxFifo = serial_get_rxfifo(serial);

            /* 1. 状态检查与更新 */
            if (rxFifo->status == SERIAL_FIFO_BUSY) // 只有在Fifo处于等待状态才处更新loadSize
                rxFifo->loadSize -= arg_size;
            rx_len = ringbuf_in(&rxFifo->rb, arg, arg_size);

            // 通知应用层RXFIFO溢出，参数是rb，大小是溢出的量，争取在err_cb中处理一部分rb中的数据，让溢出的量能够继续写入FIFO
            if (!rx_len || rx_len < arg_size)
            {
                device_err_cb(&serial->parent, ERR_SERIAL_RXFIFO_OVERFLOW, arg_size - rx_len);
                // 假设应用层已经处理了FIFO错误，那么这里尝试再写入一次
                rx_len += ringbuf_in(&rxFifo->rb, arg + rx_len, arg_size - rx_len);
                // 如果应用层没有做任何事情，这里也不用再通知一次，没有意义，err_cb已经完成它通知应用层的使命了
            }
            data_len = ringbuf_len(&rxFifo->rb);
            if (SERIAL_IS_O_BLCK_RX(&serial->parent) && rxFifo->loadSize <= 0)
                completion_done(&rxFifo->cpt);
            device_read_cb(&serial->parent, data_len);
        }
        break;

        /* arg = NULL, arg_size = 实际发送长度，由底层驱动传入 */
        case SERIAL_EVENT_INT_TXDONE: /* 中断发送完成 */
        case SERIAL_EVENT_DMA_TXDONE: /* 发送完成 */
        {
            SerialFifo_t txFifo;
            size_t       liner_size;
            void*        tx_buf;
            while (!arg_size) {}; // TODO: assert
            txFifo = serial_get_txfifo(serial);
            /* 1. 状态更新 */
            ringbuf_update_out(&txFifo->rb, arg_size); // 更新FIFO读指针
            txFifo->loadSize -= arg_size;              // 更新FIFO加载数据大小
            if (txFifo->loadSize == 0)                 // loadSize为0，则说明一轮数据已经传输完毕
                txFifo->status = SERIAL_FIFO_IDLE;

            /*  2. 回调
                这里将回调放在transmit之前，是考虑到回调中可能会调用write接口往rb中塞数据，个人认为在操作之后再进行transmit会更灵活和安全
            */
            device_write_cb(&serial->parent, ringbuf_avail(&txFifo->rb));

            /* 3. 处理发送任务 */
            if (txFifo->status != SERIAL_FIFO_IDLE)
            {
                liner_size       = ringbuf_get_item_linear_space(&txFifo->rb, &tx_buf); // 获取FIFO的item线性空间
                txFifo->loadSize = liner_size;
                serial->interface->transmit(serial, tx_buf, liner_size);
            }
            else
            {
                if (SERIAL_IS_O_BLCK_TX(&serial->parent))
                    completion_done(&txFifo->cpt);
                else if (SERIAL_IS_O_NBLCK_TX(&serial->parent))
                    device_clr_status(&serial->parent, DEV_STATUS_BUSY_TX);
            }
        }
        break;

        case SERIAL_EVENT_ERR_OCCUR: /* 串口错误回调，待完善 */ device_err_cb(&serial->parent, (uint32_t)arg, 0); break;
        default: return AWLF_ERROR_PARAM;
    }
    return AWLF_OK;
}
