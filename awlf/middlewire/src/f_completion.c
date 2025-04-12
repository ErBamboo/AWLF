#include "f_completion.h"


/*  1、挂起/阻塞自身，等待被唤醒
    2、简单判断因为超时还是DONE而被唤醒，返回统一错误码
    note: 由于一些设计问题，部分RTOS在进入该函数时，可能需要额外判断Completion的状态
*/
static awlf_ret_t _portCompletionWait(completion_t c, size_t timeout)
{
    /* freertos中采用任务通知形式，需要保证configTASK_NOTIFICATION_ARRAY_ENTRIES >= 2，
        因为"通道0"是FREERTOS的默认通道，用于任务间通知。
        为避免冲突，完成量使用"通道1"进行通知 。
        （一个任务至少可以获取32种完成量，绰绰有余）。
    */
#ifdef USED_FREERTOS
    uint32_t compWaitEvent  = c->comp_event;        // 初始化中传入的事件（通常是外设事件）
    uint32_t compWaitValue = 0;
    uint32_t isCompWait = xTaskGenericNotifyWait(NotifyCompletionIndex, 0, compWaitEvent, &compWaitValue, timeout);
    if(isCompWait && (compWaitValue & compWaitEvent))
        return AWLF_OK;         // 被通知唤醒
    else
        return AWLF_ERROR_TIMEOUT;      // 超时唤醒
#endif
}

/* 对等待方进行恢复、通知等操作，严禁阻塞 */
static awlf_ret_t _portCompletionDone(completion_t c)
{
#ifdef USED_FREERTOS
    long isNeedSwitch = 0;
    if(f_check_is_in_isr())
    {
        xTaskGenericNotifyFromISR(c->wait_thread, NotifyCompletionIndex, c->comp_event, eSetBits, NULL, &isNeedSwitch);
        if(isNeedSwitch)
            portYIELD_FROM_ISR(isNeedSwitch);
    }
    else
        xTaskGenericNotify(c->wait_thread, NotifyCompletionIndex, c->comp_event, eSetBits, NULL);
#endif
    return AWLF_OK;
}

awlf_ret_t f_completion_wait(completion_t c, size_t timeout)
{
    uint32_t            primask;
    port_task_handle_t  current_port_thread;
    /* 1. 参数检查 */
    if(!c)
        return AWLF_ERROR_PARAM;
    if(f_check_is_in_isr()) 
        while(1) {
            // 禁止在中断中等待！！
        }
    
    /* 2. 获取当前线程和关中断 */
    current_port_thread = f_task_get_current_handle();
    primask = f_hw_disable_irq();
    if(c->is_done != COMP_DONE)
    {
        if(c->wait_thread != NULL)
            while(1) {
                // 同一时间同一个完成量仅能被一个线程等待！！有一对多需求请使用其他IPC对象
            }
        c->wait_thread = current_port_thread;
        c->is_done = COMP_WAIT;
        f_hw_enable_irq(primask);
        if(_portCompletionWait(c, timeout) != AWLF_OK)
        {
            // 如果done了之后没有立即被唤醒（调度延迟），反而被超时被唤醒，此时需要判断状态来确定是否完成
            if(c->is_done == COMP_DONE)
                c->is_done = COMP_INIT;
            else
                return AWLF_ERROR_TIMEOUT;
        }
		else
			c->is_done = COMP_INIT;
        return AWLF_OK;
    }
    f_hw_enable_irq(primask);
    return AWLF_OK;
}

awlf_ret_t f_completion_done(completion_t c)
{
    uint32_t   primask;
    awlf_ret_t ret;
    if(!c) return AWLF_ERROR_PARAM;
    primask = f_hw_disable_irq();

    /* 存在一种情况：多次通知，但是等待方未来得及响应，此时不重复操作 */
    if(c->is_done == COMP_DONE)
    {
        f_hw_enable_irq(primask);
        return AWLF_ERROR_BUSY;
    }
    
    port_task_handle_t task = c->wait_thread;
    if(task) 
    {
        if(_portCompletionDone(c) != AWLF_OK)
        {
            f_hw_enable_irq(primask);
            ret = AWLF_ERROR_EMPTY;
        }
        c->wait_thread = NULL;
        ret = AWLF_OK;
    }
    else
    {
        f_hw_enable_irq(primask);
        ret = AWLF_ERROR_EMPTY;
    }
    c->is_done = COMP_DONE;
    f_hw_enable_irq(primask);
    return ret;
}

void f_completion_init(completion_t c, uint32_t event) 
{
    f_enter_critical();
    c->is_done     = COMP_INIT;
    c->comp_event = event;
    f_exit_critical();
}


