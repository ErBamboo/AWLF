 #ifndef __CORE_CONFIG_H__
#define __CORE_CONFIG_H__

#define FRAME_WITH_OS     1

#if (FRAME_WITH_OS)
#define USED_FREERTOS
    //#define USED_RTTHREAD
#endif

#define configIrqTree     0  // 启用中断树
#define configDEV_METHOD  1  // 启用设备方法(也就是结构体内置的函数指针)

#endif