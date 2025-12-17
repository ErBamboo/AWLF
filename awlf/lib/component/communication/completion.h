#ifndef __COMPLETION_H__
#define __COMPLETION_H__

/*** platfrom ***/
#include "FreeRTOS.h" // TODO: 引入posix兼容层
#include "task.h"
/*** awlf ***/
#include "core/awlf_def.h"

/**** 注意！每个完成量对象能且仅能被一个线程等待！！相对应的功能请使用其他ITC对象！！！ */

/** 完成量在不同rtos中的实现应该遵守两个原则：
 * 1、保证在触发等待（wait）之前，若完成量已经释放（done），线程会立即响应，而不会继续阻塞等待下一次释放（done）。
 *    完成量对象应该在等待（wait）之前，必须先初始化（init）。
 * 2、完成量应该结合不同rtos的特性，选择效率最高的一种同步方式来实现（如freertos中的任务通知）。
 **/

#define WAIT_FOREVER 0xffffffff

typedef enum
{
    COMP_INIT,
    COMP_WAIT,
    COMP_DONE
} comp_status_e;

#define NotifyCompletionIndex (1)
#define CompEvent_SerialRx    (1 << 0)
#define CompEvent_SerialTx    (1 << 1)

#define CompEvent_CanRx       (1 << 2)
#define CompEvent_CanTx       (1 << 3)

typedef struct Completion* Completion_t;
typedef struct Completion
{
    TaskHandle_t           wait_thread;
    volatile comp_status_e status;
    uint32_t               comp_event;
} Completion_s;

AwlfRet_e completion_wait(Completion_t c, size_t timeout);
AwlfRet_e completion_done(Completion_t c);
void      completion_init(Completion_t c, uint32_t event);

#endif /* __COMPLETION_H__ */
