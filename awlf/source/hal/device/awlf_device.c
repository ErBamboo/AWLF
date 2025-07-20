#include <string.h>
#include "hal/device/core/awlf_device.h"
#include "awlf/awlf_api.h"

static LIST_HEAD(dev_list);

inline static uint8_t IS_DEV_OTYPE_NOT_SET(device_t dev, oparam_e oparams)
{
    return (oparams & (~dev->__priv.oparams));
}

inline uint32_t DEV_GET_REGPARAMS(device_t dev)
{
    return dev->__priv.regparams;
}

inline uint32_t DEV_GET_OPARAMS(device_t dev)
{
    return dev->__priv.oparams;
}

inline void SET_DEV_STATUS(device_t dev, uint32_t _status)
{
    __SET_FLAG(dev->__priv.status, (_status));
}

inline void CLR_DEV_STATUS(device_t dev, uint32_t _status)
{
    __CLR_FLAG(dev->__priv.status, (_status));
}

inline uint32_t IS_DEV_STATUS_SET(device_t dev, uint32_t _status)
{
    return __IS_FLAG_SET(dev->__priv.status, (_status));
}

device_t device_find(char* name)
{
    uint32_t    primask;
    device_t dev_pos = NULL;
    if(!name) return NULL;
    primask = awlf_hw_disable_irq();
    list_for_each_entry(dev_pos, &dev_list, list) {
        if(strcmp(dev_pos->__priv.name, name) == 0)
        {
            awlf_hw_enable_irq(primask);
            return dev_pos;    
        }
    }
    awlf_hw_enable_irq(primask);
    return NULL;
}

awlf_ret_t device_register(device_t dev, char* name, regparam_e regparams)
{
    uint32_t    primask;
    if(!dev || !name) return AWLF_ERROR_PARAM;
    primask = awlf_hw_disable_irq();
    if(device_find(name)) {
        /* 如果设备名冲突，直接返回 */
        return AWLF_ERR_CONFLICT;
    }
    INIT_LIST_HEAD(&dev->list);
    list_add_tail(&dev->list, &dev_list);
    awlf_hw_enable_irq(primask);

    dev->__priv.name = name;
    dev->__priv.regparams = regparams;
    dev->__priv.status |= DEV_STATUS_REGED;
    return AWLF_OK;
}


awlf_ret_t device_init(device_t dev)
{
    awlf_ret_t ret = AWLF_OK;
    while(!dev || !dev->ops){};
    if(dev->ops->init)
    {
        if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_INITED))
        {        
            ret = dev->ops->init(dev);
            if(ret != AWLF_OK)
            {
                //@todo log
            }
            else
            {
                SET_DEV_STATUS(dev, DEV_STATUS_INITED);
            }
        }
    }
    else
    {
        ret = AWLF_ERROR;
    }
    return AWLF_OK;
}

awlf_ret_t device_open(device_t dev, oparam_e oparams)
{
    while(!dev || !dev->ops){};
    awlf_ret_t ret = AWLF_ERROR;
    if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_INITED))
    {
        if(dev->ops->init)
            ret = dev->ops->init(dev);
        if(ret != AWLF_OK)
        {
            //@todo log
            return ret;
        }
        SET_DEV_STATUS(dev, DEV_STATUS_INITED);
    }
    
    if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_OPENED) || IS_DEV_OTYPE_NOT_SET(dev, oparams))
    {
        if(dev->ops->open)
            ret = dev->ops->open(dev, oparams);
        if(ret != AWLF_OK)
        {
            //@todo log
            return ret;
        }
        SET_DEV_STATUS(dev, DEV_STATUS_OPENED);
        dev->__priv.oparams |= oparams;
    }

    return AWLF_OK;
}


size_t device_read(device_t dev, void* pos, void *data, size_t len)
{
    while(!dev || !dev->ops){};
    if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_OPENED))
    {
        return 0;
    }
    if(dev->ops->read)
        return dev->ops->read(dev, pos, data, len);
    return 0;
}

size_t device_write(device_t dev, void* pos, void *data, size_t len)
{
    while(!dev || !dev->ops){};
    if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_OPENED))
    {
        return 0;
    }
    if(dev->ops->write)
        return dev->ops->write(dev, pos, data, len);
    return 0;
}

awlf_ret_t device_ctrl(device_t dev, size_t cmd, void* args)
{
    while(!dev || !dev->ops){};
    if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_OPENED))
    {
        return AWLF_ERROR;
    }
    if(dev->ops->control)
        return dev->ops->control(dev, cmd, args);
    return AWLF_OK;
}

awlf_ret_t device_close(device_t dev)
{
    while(!dev || !dev->ops){};
    if(!IS_DEV_STATUS_SET(dev, DEV_STATUS_OPENED))
    {
        return AWLF_ERROR;
    }
    if(dev->ops->close)
        return dev->ops->close(dev);
    return AWLF_OK;
}

void device_set_rd_cb(device_t dev, void (*callback)(device_t dev, void* params, size_t paramsz))
{
    while(!dev){};
    dev->__priv.rd_callback = callback;
}

void device_set_wd_cb(device_t dev, void (*callback)(device_t dev, void* params, size_t paramsz))
{
    while(!dev){};
    dev->__priv.wd_callback = callback;
}

void device_set_err_cb(device_t dev, void (*callback)(device_t dev, uint16_t errcode))
{
    while(!dev){};
    dev->__priv.err_callback = callback;
}
