#include "component/driver/core/device.h"
#include "core/awlf_interrupt.h"
#include <string.h>

static LIST_HEAD(gDevList);

Device_t device_find(char* name)
{
    uint32_t primask;
    Device_t dev_pos = NULL;
    if (!name)
        return NULL;
    primask = awlf_hw_disable_interrupt();
    list_for_each_entry(dev_pos, &gDevList, list)
    {
        if (strcmp(dev_pos->__priv.name, name) == 0)
        {
            awlf_hw_restore_interrupt(primask);
            return dev_pos;
        }
    }
    awlf_hw_restore_interrupt(primask);
    return NULL;
}

AwlfRet_e device_register(Device_t Dev, char* name, uint32_t regparams)
{
    uint32_t intLevle;
    if (!Dev || !name)
        return AWLF_ERROR_PARAM;
    intLevle = awlf_hw_disable_interrupt();
    if (device_find(name))
    {
        /* 如果设备名冲突，直接返回 */
        // TODO: log
        awlf_hw_restore_interrupt(intLevle);
        return AWLF_ERR_CONFLICT;
    }
    INIT_LIST_HEAD(&Dev->list);
    list_add_tail(&Dev->list, &gDevList);
    awlf_hw_restore_interrupt(intLevle);

    Dev->__priv.name           = name;
    Dev->__priv.regparams      = regparams;
    Dev->__priv.refCount       = 0;
    Dev->__priv.c_flags        = 0;
    Dev->__priv.oparams        = 0;
    Dev->__priv.read_callback  = NULL;
    Dev->__priv.write_callback = NULL;
    Dev->__priv.err_callback   = NULL;

    return AWLF_OK;
}

AwlfRet_e device_init(Device_t Dev)
{
    AwlfRet_e ret = AWLF_OK;
    while (!Dev || !Dev->interface) {};
    if (Dev->interface->init)
    {
        if (!device_check_status(Dev, DEV_STATUS_INITED))
        {
            ret = Dev->interface->init(Dev);
            if (ret != AWLF_OK)
            {
                // TODO:  LOG ERR
            }
            else
            {
                device_set_status(Dev, DEV_STATUS_INITED);
            }
        }
    }
    else
    {
        ret = AWLF_ERROR;
        // TODO: LOG ERR
    }
    return ret;
}

AwlfRet_e device_open(Device_t Dev, uint32_t oparams)
{
    while (!Dev || !Dev->interface) {}; // TODO: assert
    AwlfRet_e ret = AWLF_ERROR;
    if (!device_check_status(Dev, DEV_STATUS_INITED))
    {
        if (Dev->interface->init)
            ret = device_init(Dev);
        if (ret != AWLF_OK)
        {
            // TODO: LOG ERR
            return ret;
        }
    }

    // TODO: 区分仅能被Open一次的设备和能被多次Open的设备
    if (!device_check_status(Dev, DEV_STATUS_OPENED))
    {
        if (Dev->interface->open)
            ret = Dev->interface->open(Dev, oparams);
        if (ret != AWLF_OK)
        {
            // TODO: LOG ERR
            return ret;
        }
        Dev->__priv.refCount++;
        device_set_status(Dev, DEV_STATUS_OPENED);
        // Dev->__priv.oparams |= oparams;  // oparams由具体实现赋值
    }

    return AWLF_OK;
}

size_t device_read(Device_t Dev, void* pos, void* data, size_t len)
{
    while (!Dev || !Dev->interface || !data || !len) {}; // TODO: assert
    if (!device_check_status(Dev, DEV_STATUS_OPENED) || device_check_status(Dev, DEV_STATUS_SUSPEND))
    {
        return 0;
    }
    if (Dev->interface->read)
        return Dev->interface->read(Dev, pos, data, len);
    return 0;
}

size_t device_write(Device_t Dev, void* pos, void* data, size_t len)
{
    while (!Dev || !Dev->interface || !data || !len) {}; // TODO: assert
    if (!device_check_status(Dev, DEV_STATUS_OPENED) || device_check_status(Dev, DEV_STATUS_SUSPEND))
    {
        return 0;
    }
    if (Dev->interface->write)
        return Dev->interface->write(Dev, pos, data, len);
    return 0;
}

AwlfRet_e device_ctrl(Device_t Dev, size_t cmd, void* args)
{
    while (!Dev || !Dev->interface) {};
    if (!device_check_status(Dev, DEV_STATUS_OPENED))
    {
        return AWLF_ERROR;
    }
    if (Dev->interface->control)
        return Dev->interface->control(Dev, cmd, args);
    return AWLF_OK;
}

AwlfRet_e device_close(Device_t Dev)
{
    AwlfRet_e ret = AWLF_OK;
    while (!Dev || !Dev->interface || !Dev->interface->close) {}; // TODO: assert
    if (!device_check_status(Dev, DEV_STATUS_OPENED))
    {
        // @TODO: log
        return AWLF_ERROR;
    }
    if (--Dev->__priv.refCount == 0)
    {
        ret = Dev->interface->close(Dev);
        if (ret == AWLF_OK)
        {
            Dev->__priv.status = DEV_STATUS_CLOSED;
        }
        else
        {
            // @TODO: log err
        }
    }
    return ret;
}
