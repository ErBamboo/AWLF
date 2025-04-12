#ifndef __COMPLETION_H__
#define __COMPLETION_H__

#include "f_task.h"
#include "f_def.h"
#include "f_api.h"

/**** 注意！每个完成量对象能且仅能被一个线程等待！！相对应的功能请使用其他IPC对象！！！ */

/** 完成量在不同rtos中的实现应该遵守两个原则：
 * 1、保证在触发等待（wait）之前，若完成量已经释放（done），线程会立即响应，而不会继续阻塞等待下一次释放（done）。
 *    完成量对象应该在等待（wait）之前，必须先初始化（init）。
 * 2、完成量应该结合不同rtos的特性，选择效率最高的一种同步方式来实现（如freertos中的任务通知）。
 **/


#define COMP_INIT 0
#define COMP_WAIT 1
#define COMP_DONE 2

#define CompEvent_SerialRx    1 << 0
#define CompEvent_SerialTx    1 << 1


typedef struct completion *completion_t;
typedef struct completion {
    port_task_handle_t wait_thread;
    uint8_t  is_done;
    uint32_t comp_event;
} completion_s;

awlf_ret_t  f_completion_wait(completion_t c, size_t timeout);
awlf_ret_t  f_completion_done(completion_t c);
void        f_completion_init(completion_t c, uint32_t event);

#endif  /* __COMPLETION_H__ */
