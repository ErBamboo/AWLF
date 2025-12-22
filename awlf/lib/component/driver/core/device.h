/**
 * @file   device.h
 * @author Yu hao
 * @brief  设备驱动模型接口定义
 * @version 0.1
 * @date 2025-11-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __AWLF_DEVICE_H__
#define __AWLF_DEVICE_H__

#include "data_struct/corelist.h"
#include "aw_core/awlf_def.h"
#include "aw_core/awlf_atomic.h"

typedef struct DevInterface* DevInterface_t;
typedef struct Device*       Device_t;

typedef struct DevInterface
{
    AwlfRet_e (*init)(Device_t dev);
    AwlfRet_e (*open)(Device_t dev, uint32_t oparam);
    AwlfRet_e (*close)(Device_t dev);
    size_t (*read)(Device_t dev, void* ctrlInfo, void* data, size_t len);
    size_t (*write)(Device_t dev, void* ctrlInfo, void* data, size_t len);
    AwlfRet_e (*control)(Device_t dev, size_t cmd, void* args);
} DevInterface_s;

typedef struct DevAttr* DevAttr_t;
typedef struct DevAttr
{
    char*    name;      // 设备名称
    uint32_t oparams;   // 打开方式
    uint32_t regparams; // 设备注册标志
    uint32_t refCount;  // 引用计数
    uint32_t c_flags;   // TODO: 设备控制标志
    uint32_t status;    // 设备状态

    void* param;                                                                       // 设备参数
    void (*read_callback)(Device_t dev, void* param, size_t paramsz);                  // 读数据回调
    void (*write_callback)(Device_t dev, void* param, size_t paramsz);                 // 写数据回调
    void (*err_callback)(Device_t dev, uint32_t errcode, void* param, size_t paramsz); // 错误回调
} DevAttr_s;

typedef struct Device
{
    DevInterface_t   interface;
    DevAttr_s        __priv; // protected，非调试需要，禁止用户访问
    void*            handle;
    struct list_head list;
} Device_s;

/* 设备操作方法 */
Device_t  device_find(char* name);
AwlfRet_e device_init(Device_t dev);
AwlfRet_e device_open(Device_t dev, uint32_t oparam);
AwlfRet_e device_close(Device_t dev);
size_t    device_read(Device_t dev, void* pos, void* data, size_t len);
size_t    device_write(Device_t dev, void* pos, void* data, size_t len);
AwlfRet_e device_ctrl(Device_t dev, size_t cmd, void* args);
AwlfRet_e device_register(Device_t dev, char* name, uint32_t regparams);

/* 注册回调函数相关API */
static inline void device_set_read_cb(Device_t dev, void (*callback)(Device_t dev, void* params, size_t paramsz))
{
    while (!dev) {}; // TODO: assert
    dev->__priv.read_callback = callback;
}

static inline void device_set_write_cb(Device_t dev, void (*callback)(Device_t dev, void* params, size_t paramsz))
{
    while (!dev) {}; // TODO: assert
    dev->__priv.write_callback = callback;
}

static inline void device_set_err_cb(Device_t dev, void (*callback)(Device_t dev, uint32_t errcode, void* params, size_t paramsz))
{
    while (!dev) {}; // TODO: assert
    dev->__priv.err_callback = callback;
}

static inline void device_set_param(Device_t dev, void* param)
{
    while (!dev) {}; // TODO: assert
    dev->__priv.param = param;
}

static inline void device_read_cb(Device_t dev, size_t paramsz)
{
    if (dev->__priv.read_callback)
        dev->__priv.read_callback(dev, dev->__priv.param, paramsz);
}

static inline void device_write_cb(Device_t dev, size_t paramsz)
{
    if (dev->__priv.write_callback)
        dev->__priv.write_callback(dev, dev->__priv.param, paramsz);
}

static inline void device_err_cb(Device_t dev, uint32_t errcode, size_t paramsz)
{
    if (dev->__priv.err_callback)
        dev->__priv.err_callback(dev, errcode, dev->__priv.param, paramsz);
}

/* 参数、状态管理API */
static inline uint32_t device_get_oparams(Device_t dev)
{
    return dev->__priv.oparams;
}

static inline uint32_t device_get_cflags(Device_t dev)
{
    return dev->__priv.c_flags;
}

static inline void device_set_cflags(Device_t dev, uint32_t c_flags)
{
    dev->__priv.status |= c_flags;
}

static inline void device_set_status(Device_t dev, uint32_t _status)
{
    __ATOMIC_SET_FLAG(dev->__priv.status, (_status));
}

static inline void device_clr_status(Device_t dev, uint32_t _status)
{
    __ATOMIC_CLR_FLAG(dev->__priv.status, (_status));
}

// TODO: 或许Check行为应该由不同模块自定义，device只负责提供get方法
static inline uint32_t device_check_status(Device_t dev, uint32_t _status)
{
    return __ATOMIC_IS_FLAG_SET(dev->__priv.status, (_status));
}

static inline uint32_t device_get_regparams(Device_t dev)
{
    return dev->__priv.regparams;
}

static inline char* device_get_name(Device_t dev)
{
    return dev->__priv.name;
}

#endif // __AWLF_DEVICE_H__
