#ifndef __TASK_H__
#define __TASK_H__

#include "f_def.h"
#include "f_api.h"

#if defined(USED_FREERTOS)
typedef TaskHandle_t port_task_handle_t;
#define NotifyTaskIndex        0         // 任务通知
#define NotifyCompletionIndex  1         // 完成量通知
#elif defined(USED_RTTHREAD)
typedef rt_thread_t port_task_handle_t;
#endif


typedef struct f_task* f_task_t;
typedef struct f_task {
    port_task_handle_t port_handle;
    struct list_head list;
} f_task_s;

awlf_ret_t f_task_create 
(  
    f_task_t task_handle,
    port_task_handle_t* port_task_handle,   
    const char* name,
    size_t priority, 
    size_t stack_size,
    void (*task_func)(void*), 
    void* arg                            
);

#if (FRAME_WITH_OS)
#define WAIT_FOREVER                      portWAIT_TIME_FOREVER
#define f_task_get_handle_from_name(name) portGetTaskSelfFromName(name)
#define f_task_get_current_handle()       portGetTaskSelf()
#define f_check_is_in_isr()               portCheckIsInISR()
#define f_task_suspend(port_task_handle)  portSuspendTaskFunc(port_task_handle)
#define f_task_resume(port_task_handle)   portResumeTaskFunc(port_task_handle)
#elif
#define f_task_get_current_handle()       NULL
#define f_check_is_in_isr()               0

#define f_task_suspend(port_task_handle)  
#define f_task_resume(port_task_handle)   
#endif

#endif