#include "hal/device/hal_serial.h"
#include <stdlib.h>
#include "awlf/awlf_api.h"
#include "itc/completion.h"

/************************** PRIVATE FUNC ********************************************************/

/* 无FIFO情况下的 TX 函数*/
static size_t _serial_tx_poll(device_t dev, void* pos, void *data, size_t len)
{
    size_t length;
    hal_serial_t serial = (hal_serial_t)dev;
    uint8_t* _data = (uint8_t*)data;
    if(IS_DEV_STATUS_SET(dev, DEV_STATUS_BUSY_TX))
    {
        // @todo: log txbusy
        return 0;
    }
    SET_DEV_STATUS(dev, DEV_STATUS_BUSY_TX);
    for(length = 0; length < len; length++)
    {
        if(serial->interface->putByte(serial, _data[length]) != AWLF_OK)
            break;
    }
    CLR_DEV_STATUS(dev, DEV_STATUS_BUSY_TX);
    return length;
}

static size_t _serial_rx_poll(device_t dev, void* pos, uint8_t *buf, size_t len)
{
    size_t length;
    hal_serial_t serial = (hal_serial_t)dev;
    if(IS_DEV_STATUS_SET(dev, DEV_STATUS_BUSY_RX))
    {
        // @todo: log txbusy
        return 0;
    }
    SET_DEV_STATUS(dev, DEV_STATUS_BUSY_RX);
    for(length = 0; length < len; length++)
    {
        *(buf++) = serial->interface->getByte(serial);
    }
    CLR_DEV_STATUS(dev, DEV_STATUS_BUSY_RX);
    return length;
}

/**
 * @brief 串口阻塞发送 内部框架实现
 *
 * @param dev 串口设备
 * @param pos 串口不使用该参数
 * @param data 应用层数据包指针
 * @param len  应用层数据包长度
 * @return size_t 实际发送长度
 */
static size_t _serial_tx_block(device_t dev, void* pos, void *data, size_t len)
{
    hal_serial_t   serial;
    serial_fifo_t  tx_fifo;
    size_t         sended = 0;
    serial = (hal_serial_t)dev;
    tx_fifo = serial->__priv.tx_fifo;
    // fifo检查放在write函数中，提高复用
    if(IS_DEV_STATUS_SET(dev, DEV_STATUS_BUSY_TX))
        return 0;

    do
    {
        tx_fifo->load_size = ringbuf_in(&tx_fifo->rb, data + sended, len);   // 将数据写入FIFO中，由硬件读出并发送
        if (tx_fifo->load_size > 0) {
            serial->interface->transmit(serial, data + sended, tx_fifo->load_size);
            len -= tx_fifo->load_size;
            sended += tx_fifo->load_size;
            completion_wait(&tx_fifo->cpt, WAIT_FOREVER);
        }
    } while (len); // 当发送完成后退出
    tx_fifo->load_size = 0;
    CLR_DEV_STATUS(dev, DEV_STATUS_BUSY_TX);
    return sended;
}

/* 串口非阻塞发送 */
static size_t _serial_tx_nonblock(device_t dev, void* pos, void *data, size_t len)
{
    hal_serial_t   serial;
    serial_fifo_t  tx_fifo;
    uint32_t       primask;
    size_t         sended = 0;
    void*          tx_ptr = NULL;

    serial = (hal_serial_t)dev;
    tx_fifo = serial->__priv.tx_fifo;
    // fifo检查放在write函数中，提高复用
    primask = awlf_hw_disable_irq();
    sended = ringbuf_in(&tx_fifo->rb, data, len);

    /*  load_size 将会在发送完成后被清零，若不为0则说明有数据正在发送，
        此时不可以重新transmit，将数据放入FIFO即可
    */
    if(tx_fifo->load_size)  // 数据正在发送
    {
        awlf_hw_enable_irq(primask);
        return sended;
    }
    else  // load_size == 0
    {
        awlf_hw_enable_irq(primask);
        /* 发送数据 */
        if (sended > 0)
        {
            tx_fifo->load_size = ringbuf_get_linear_space(&tx_fifo->rb, &tx_ptr);
            if (tx_fifo->load_size > 0)
                serial->interface->transmit(serial, tx_ptr, tx_fifo->load_size);
        }
    }
    return sended;
}

static awlf_ret_t _serial_tx_enable(device_t dev)
{
    hal_serial_t serial = (hal_serial_t)dev;
    serial_fifo_t tx_fifo = serial->__priv.tx_fifo;
    size_t ctrl_arg = 0;
    // 无缓存
    if(serial->cfg.txbufsz == 0)
    {
        // 串口无缓存只能使用poll方式
        return AWLF_OK;
    }

    /* 底层未填充 FIFO */
    if(!tx_fifo)
    {
        /*  当前框架下，除了poll之外所有发送方式都会配置rb，无论阻塞非阻塞。
            后续若是发现不合适的话可以再细分，目前暂不支持。
            但是无论如何，建议非阻塞发送都使用rb。
        */
        if(serial->cfg.txbufsz < SERIAL_MIN_TX_BUFSZ)  // @todo: log
            serial->cfg.txbufsz = SERIAL_MIN_TX_BUFSZ;

        tx_fifo = (serial_fifo_t)malloc(sizeof(serial_fifo_s));
        while(!tx_fifo) {}; // @todo: assert
        ringbuf_alloc(&tx_fifo->rb, sizeof(uint8_t), serial->cfg.txbufsz);
        if(!tx_fifo->rb.buf)
        {
            free(tx_fifo);
            while(1){}; // @todo: assert
        }
        serial->__priv.tx_fifo = tx_fifo;
    }

    if(DEV_GET_OTYPE(dev) & OTYPE_BLOCKING_TX)
        completion_init(&tx_fifo->cpt, CompEvent_SerialTx);

    if(DEV_GET_REGFLAGS(dev) & REG_DMA_TX)  // 注册了DMA发送
        ctrl_arg = REG_DMA_TX;

    else if(DEV_GET_REGFLAGS(dev) & REG_IRQ_TX)  // 注册了中断发送
        ctrl_arg = REG_IRQ_TX;
    else
        return AWLF_ERROR_NOT_SUPPORT;

    serial->interface->control(serial, F_DEV_CTRL_CFG, (void*)ctrl_arg);

    return AWLF_OK;
}

static awlf_ret_t _serial_rx_enable(device_t dev)
{
    hal_serial_t serial;
    serial_fifo_t rx_fifo;
    size_t ctrl_arg;

    if(DEV_GET_REGFLAGS(dev) & REG_DMA_RX)  // 注册了DMA接收
        ctrl_arg = REG_DMA_RX;
    else if(DEV_GET_REGFLAGS(dev) & REG_IRQ_RX)  // 注册了中断接收
        ctrl_arg = REG_IRQ_RX;
    else
        return AWLF_ERROR_NOT_SUPPORT;

    serial = (hal_serial_t)dev;
    rx_fifo = serial->__priv.rx_fifo;

    if(serial->cfg.rxbufsz == 0)
    {
        // 无缓存，串口只能轮询读取
        return AWLF_OK;
    }

    if(!rx_fifo)
    {
        if(serial->cfg.rxbufsz < SERIAL_MIN_RX_BUFSZ)
            serial->cfg.rxbufsz = SERIAL_MIN_RX_BUFSZ;

        rx_fifo = (serial_fifo_t)malloc(sizeof(serial_fifo_s));
        while(!rx_fifo) {}; // @todo: assert
        ringbuf_alloc(&rx_fifo->rb, sizeof(uint8_t), serial->cfg.rxbufsz);
        if(!rx_fifo->rb.buf)
        {
            free(rx_fifo);
            while(1){}; // @todo: assert
        }
        serial->__priv.rx_fifo = rx_fifo;
    }
    if(DEV_GET_OTYPE(dev) & OTYPE_BLOCKING_TX)
        completion_init(&rx_fifo->cpt, CompEvent_SerialRx);

    serial->interface->control(serial, F_DEV_CTRL_CFG, (void*)ctrl_arg);        
    return AWLF_OK;
}

/************************** IO API ***************************************************************/
static awlf_ret_t serial_init(device_t dev)
{
    hal_serial_t serial = (hal_serial_t)dev;
    if(!serial || !serial->interface || !serial->interface->configure)
        return AWLF_ERROR_PARAM;
    return serial->interface->configure(serial, &serial->cfg);
}

static awlf_ret_t serial_open(device_t dev, otype_e otype)
{
    hal_serial_t serial = (hal_serial_t)dev;
    if(!serial || !serial->interface || !serial->interface->control)
        return AWLF_ERROR_PARAM;

    /* 串口写方式 */
    dev->__priv.otype |= (otype & OTYPE_BLOCKING_TX) ? OTYPE_BLOCKING_TX : OTYPE_NON_BLOCKING_TX;
    /* 串口读方式 */
    dev->__priv.otype |= (otype & OTYPE_BLOCKING_RX) ? OTYPE_BLOCKING_RX : OTYPE_NON_BLOCKING_RX;

    _serial_tx_enable(dev);
    _serial_rx_enable(dev);
    return AWLF_OK;
}

static awlf_ret_t serial_ctrl(device_t dev, size_t cmd, void* arg)
{
    hal_serial_t serial = (hal_serial_t)dev;
    if(!serial || !serial->interface || !serial->interface->control)
        return AWLF_ERROR_PARAM;

    switch(cmd)
    {
        case F_DEV_CTRL_FLUSH:
            for(;;)
            {
                size_t flush_len = 0;
                void* flush_ptr;
                flush_len = ringbuf_get_linear_space(&serial->__priv.rx_fifo->rb, &flush_ptr);
                if(!flush_len)
                    break;
                _serial_tx_poll(dev, 0, flush_ptr, flush_len);
                ringbuf_update_out(&serial->__priv.rx_fifo->rb, flush_len);
            }
        break;

        default:
            serial->interface->control(serial, cmd, arg);
    }
    
    return AWLF_OK;
}

static size_t serial_write(device_t dev, void* pos, void *data, size_t len)
{
    serial_fifo_t  fifo;
    hal_serial_t   serial;
    size_t         ret_len = 0;
    if(!dev || !len || !data) return 0;
    serial = (hal_serial_t)dev;
    fifo = serial->__priv.tx_fifo;

    if(serial->cfg.txbufsz == 0)
    {
        return _serial_tx_poll(dev, pos, data, len);
    }
    else if(!fifo || !fifo->rb.buf)
    {
        //@todo: log null fifo
        return 0;
    }

    if(DEV_GET_OTYPE(dev) & OTYPE_BLOCKING_TX)
    {
        ret_len = _serial_tx_block(dev, pos, data, len);
    }
    else if(DEV_GET_OTYPE(dev) & OTYPE_NON_BLOCKING_TX)
    {
        ret_len = _serial_tx_nonblock(dev, pos, data, len);
    }
    CLR_DEV_STATUS(dev, DEV_STATUS_BUSY_TX);
    //@todo log err otype
    return ret_len;
}

static size_t serial_read(device_t dev, void* pos, void* buf, size_t len)
{
    hal_serial_t serial;
    serial_fifo_t rx_fifo;
    size_t recv_len = 0;
    size_t remaining_size = 0;
    uint32_t primsk;
    if(!dev || !buf || !len) return 0;
    serial = (hal_serial_t)dev;
    rx_fifo = serial->__priv.rx_fifo;
    if(serial->cfg.rxbufsz == 0)
    {
        return _serial_rx_poll(dev, pos, (uint8_t*)buf, len);
    }
    if(!rx_fifo || !rx_fifo->rb.buf)
    {
        // log no buf
        return 0;
    }
    if(DEV_GET_OTYPE(dev) & OTYPE_BLOCKING_RX)
    {
        while(recv_len < len)
        {
            remaining_size = len - recv_len;        // 剩余空间
            if(remaining_size)
            {
                rx_fifo->load_size = (remaining_size > ringbuf_cap(&rx_fifo->rb)) ?
                                     ringbuf_cap(&rx_fifo->rb) :
                                     remaining_size;
                completion_wait(&rx_fifo->cpt, WAIT_FOREVER);
				recv_len += ringbuf_out(&rx_fifo->rb, buf + recv_len, remaining_size);  // 读取数据
            }
        }
        rx_fifo->load_size = 0;
        return recv_len;
    }
    primsk = awlf_hw_disable_irq();
    recv_len = ringbuf_out(&rx_fifo->rb, buf, len);
    awlf_hw_enable_irq(primsk);
    return recv_len;
}

static dev_interface_s serial_interface = {
    .init   = serial_init,
    .open   = serial_open,
    .close  = AWLF_NULL,        // 可能用于：shell的关闭、低功耗等，待完成
    .control= serial_ctrl,      // 待完善
    .read   = serial_read,
    .write  = serial_write,
};

/************************** HW API ***************************************************************/

/**
 * @brief 获取接收mask - 用于特殊数据位
 *
 * @param serial 串口设备
 * @return uint32_t 接收mask
 * @note 这个函数将被底层驱动调用获取mask，该值用于统一不同数据位之间接收数据的差异。
 */
 inline uint32_t serial_get_rxmask(hal_serial_t serial)
{
    return (((uint32_t)(serial->cfg.parity == PARITY_NONE) << ((serial->cfg.databits >> 16))) | serial->cfg.databits) & 0x1FF;
}

awlf_ret_t serial_register(hal_serial_t serial, char* name, void* handle, uint32_t regflag)
{
    if(!serial || !name) return AWLF_ERROR_PARAM;
    serial->parent.handle = handle;
    serial->parent.ops = &serial_interface;
    return device_register(&serial->parent, name, regflag);
}

awlf_ret_t serial_hw_isr(hal_serial_t serial, serial_event_e event, void* arg, size_t arg_size)
{
    if(!serial || !serial->interface || !serial->interface->control)
        return AWLF_ERROR_PARAM;

    switch (event) {
        case SERIAL_EVENT_DMARXDONE:    /* DMA接收完成 */
        case SERIAL_EVENT_INT_RXDONE:   /* 中断接收完成 */
            {
                serial_fifo_t rx_fifo = serial->__priv.rx_fifo;
                size_t data_len;
                while(!rx_fifo){}; // @todo assert
                ringbuf_in(&rx_fifo->rb, arg, arg_size);
                data_len = ringbuf_len(&rx_fifo->rb);
                if(data_len >= rx_fifo->load_size)
                {
                    if(DEV_GET_OTYPE(&serial->parent) & OTYPE_BLOCKING_RX)
                        completion_done(&rx_fifo->cpt);
                }
            }
            break;

        case SERIAL_EVENT_INT_TXDONE:   /* 中断发送完成 */
            {
                serial_fifo_t tx_fifo = serial->__priv.tx_fifo;
                size_t data_len;
                uint8_t data;
                while(!tx_fifo){};  // @todo assert
                data_len = ringbuf_len(&tx_fifo->rb);
                if (ringbuf_out(&tx_fifo->rb, &data, 1) == 1) {
                    serial->interface->putByte(serial, data);
                }
                else
                {
                    if(DEV_GET_OTYPE(&serial->parent) & OTYPE_BLOCKING_TX)
                        completion_done(&tx_fifo->cpt);
                    else if(DEV_GET_OTYPE(&serial->parent) & OTYPE_NON_BLOCKING_TX)
                        tx_fifo->load_size = 0;
                    serial_ctrl(&serial->parent, F_DEV_CTRL_CLR_INT, (void*)REG_IRQ_TX);    // 考虑兼容性，此处可能下放到BSP层实现
                }
            }
            break;
        case SERIAL_EVENT_DMATXDONE:    /* DMA发送完成 arg = NULL, arg_size = 实际发送长度，由底层驱动传入*/
            {
                void* tx_ptr;
                size_t load_size = arg_size;
                serial_fifo_t tx_fifo = serial->__priv.tx_fifo;
                ringbuf_update_out(&tx_fifo->rb, load_size);
                load_size = ringbuf_get_linear_space(&tx_fifo->rb, &tx_ptr);
                if(load_size > 0)
                {
                    tx_fifo->load_size -= load_size;
                    serial->interface->transmit(serial, tx_ptr, load_size);
                }
                else if(DEV_GET_OTYPE(&serial->parent) & OTYPE_BLOCKING_TX)
                    completion_done(&tx_fifo->cpt);
                else if(DEV_GET_OTYPE(&serial->parent) & OTYPE_NON_BLOCKING_TX)
                    tx_fifo->load_size = 0;
            }
            break;
        case SERIAL_EVENT_ERR:          /* 串口错误 */

            break;
        default:
            return AWLF_ERROR_PARAM;
    }

    return AWLF_OK;
}

awlf_ret_t serial_init_fifo(hal_serial_t serial, serial_fifo_t fifo, uint8_t* buf, size_t bufsz)
{
    if(!serial || !fifo || !buf || !bufsz)
    {
        //@todo: log
        return AWLF_ERROR_PARAM;
    }
    ringbuf_init(&fifo->rb, buf, sizeof(uint8_t), bufsz);
    serial->__priv.tx_fifo = fifo;
    return AWLF_OK;
}

inline serial_fifo_t serial_get_rxfifo(hal_serial_t serial)
{
    return serial->__priv.rx_fifo;
}

/* 从串口FIFO中读取数据 */
static size_t serial_get_fifo_data(hal_serial_t serial, uint8_t *buf, size_t len)
{
    serial_fifo_t  fifo;
    if(!serial || !len || !buf) return 0;
    fifo = serial->__priv.tx_fifo;
    if(!fifo || !fifo->rb.buf)
    {
        //@todo: log
        return 0;
    }
    return ringbuf_out(&fifo->rb, buf, len);
}

