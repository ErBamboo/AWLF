#ifndef __AWLF_DEVICE_H__
#define __AWLF_DEVICE_H__

#include <stddef.h>
#include <stdint.h>
#include "data_struct/corelist.h"
#include "awlf/awlf_def.h"

typedef struct dev_interface* dev_interface_t;
typedef struct device* device_t;

typedef struct dev_interface {
    awlf_ret_t (*init)   (device_t dev);
    awlf_ret_t (*open)   (device_t dev, otype_e otype);
    awlf_ret_t (*close)  (device_t dev);
    size_t     (*read)   (device_t dev, void* pos, void *data, size_t len);
    size_t     (*write)  (device_t dev, void* pos, void *data, size_t len);
    awlf_ret_t (*control)(device_t dev, size_t cmd, void* args);
} dev_interface_s;

typedef struct dev_attr* dev_attr_t;
typedef struct dev_attr {
    char*        name;      // 设备名称
    otype_e      otype;     // 打开方式
    uint32_t     reg_falgs; // 设备注册标志
    dev_status_e status;    // 设备状态
    void (*rxdone_callback)(device_t dev, void* itembuf, size_t itemsz);
    void (*txdone_callback)(device_t dev, void* itembuf, size_t itemsz);
    void (*err_callback)(device_t dev, uint16_t errcode);
} dev_attr_s;

typedef struct device {
    dev_interface_t  ops;
    dev_attr_s       __priv;        // protected
    void*            handle;
    struct list_head list;
} device_s;

awlf_ret_t device_init(device_t dev);
awlf_ret_t device_open(device_t dev, otype_e otype);
awlf_ret_t device_close(device_t dev);
size_t device_read(device_t dev, void* pos, void *data, size_t len);
size_t device_write(device_t dev, void* pos, void *data, size_t len);
awlf_ret_t device_ctrl(device_t dev, size_t cmd, void* args);
device_t device_find(char* name);

void device_set_rx_callback(device_t dev, void (*callback)(device_t dev, void* itembuf, size_t itemsz));
void device_set_tx_callback(device_t dev, void (*callback)(device_t dev, void* itembuf, size_t itemsz));
void device_set_err_callback(device_t dev, void (*callback)(device_t dev, uint16_t errcode));

awlf_ret_t device_register(device_t dev, char* name, uint32_t regflag);

uint32_t DEV_GET_REGFLAGS(device_t dev);
uint32_t DEV_GET_OTYPE(device_t dev);
void SET_DEV_STATUS(device_t dev, uint32_t _status);
void CLR_DEV_STATUS(device_t dev, uint32_t _status);
uint32_t IS_DEV_STATUS_SET(device_t dev, uint32_t _status);


#endif // __AWLF_DEVICE_H__
