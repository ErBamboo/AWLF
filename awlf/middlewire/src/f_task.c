#include "f_task.h"

/* Function to initialize a task */
static inline void init_task(f_task_t task) {
    task->port_handle = NULL;
    INIT_LIST_HEAD(&task->list);
}

/* 注意，rtthread中任务优先级是与freertos相反的 */
awlf_ret_t f_task_create(f_task_t task_handle,
                            port_task_handle_t* port_task_handle,   
                            const char* name,
                            size_t priority, 
                            size_t stack_size,
                            void (*task_func)(void*), 
                            void* arg
                    ) 
{

    awlf_ret_t result = AWLF_ERROR;

    if (task_handle == NULL) {
        return result;
    }
    init_task(task_handle);

    /* freertos */
#ifdef USED_FREERTOS
    /*<!>  freertos code start  <!>*/ 
    BaseType_t task_result = xTaskCreate(task_func, name, stack_size, arg, priority, port_task_handle);
    if (task_result == pdPASS)
    {
        task_handle->port_handle = *port_task_handle;
        init_task(task_handle);
        result = AWLF_OK;
    }
    /*<!>  freertos code end  <!>*/
#elif defined(USED_RTTHREAD)

#endif

    return result;
}

