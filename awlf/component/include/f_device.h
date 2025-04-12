#ifndef __F_DEVICE_H__
#define __F_DEVICE_H__

#include "awlf_include.h"

typedef struct dev_interface* dev_interface_t;
typedef struct f_device* f_device_t;


typedef struct dev_interface {
    awlf_ret_t (*init)   (f_device_t dev);
    awlf_ret_t (*open)   (f_device_t dev, otype_e otype);
    awlf_ret_t (*close)  (f_device_t dev);
    size_t     (*read)   (f_device_t dev, void* pos, void *data, size_t len);
    size_t     (*write)  (f_device_t dev, void* pos, void *data, size_t len);
    size_t     (*control)(f_device_t dev, size_t cmd, void* args);
} dev_interface_s;

typedef struct dev_attr* dev_attr_t;
typedef struct attr {
    char*        name;      // 设备名称
    otype_e      otype;     // 打开方式
    uint32_t     reg_falgs; // 设备注册标志
    dev_status_e status;    // 设备状态
    void (*rxdone_callback)(f_device_t dev, void* itembuf, size_t itemsz);
    void (*txdone_callback)(f_device_t dev, void* itembuf, size_t itemsz);
    void (*err_callback)(f_device_t dev, uint16_t errcode);
} dev_attr_s;

typedef struct f_device {
    dev_interface_t  ops;
    dev_attr_s       __priv;        // protected
    void*            handle;
    struct list_head list;
} f_device_s;

awlf_ret_t device_init(f_device_t dev);
awlf_ret_t device_open(f_device_t dev, otype_e otype);
awlf_ret_t device_close(f_device_t dev);
size_t device_read(f_device_t dev, void* pos, void *data, size_t len);
size_t device_write(f_device_t dev, void* pos, void *data, size_t len);
awlf_ret_t device_ctrl(f_device_t dev, size_t cmd, void* args);
f_device_t device_find(char* name);
void device_set_rx_callback(f_device_t dev, void (*callback)(f_device_t dev, void* itembuf, size_t itemsz));
void device_set_tx_callback(f_device_t dev, void (*callback)(f_device_t dev, void* itembuf, size_t itemsz));
void device_set_err_callback(f_device_t dev, void (*callback)(f_device_t dev, uint16_t errcode));

awlf_ret_t device_register(f_device_t dev, char* name, uint32_t regflag);

uint32_t DEV_GET_REGFLAGS(f_device_t dev);
uint32_t DEV_GET_OTYPE(f_device_t dev);
void SET_DEV_STATUS(f_device_t dev, uint32_t _status);
void CLR_DEV_STATUS(f_device_t dev, uint32_t _status);
uint32_t IS_DEV_STATUS_SET(f_device_t dev, uint32_t _status);


#endif // __F_DEVICE_H__
