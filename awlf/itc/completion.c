#include "inc/completion.h"  
#include "awlf_api.h"

/*  1、挂起/阻塞自身，等待被唤醒
    2、简单判断因为超时还是DONE而被唤醒，返回统一错误码
    note: 由于一些设计问题，部分RTOS在进入该函数时，可能需要额外判断Completion的状态
*/
static awlf_ret_t CompletionWait(completion_t c, size_t timeout)
{
    /* freertos中采用任务通知形式，需要保证configTASK_NOTIFICATION_ARRAY_ENTRIES >= 2，
        因为"通道0"是FREERTOS的默认通道，用于任务间通知。
        为避免冲突，完成量使用"通道1"进行通知 。
        （一个任务至少可以获取32种完成量，绰绰有余）。
    */
    uint32_t compWaitEvent  = c->comp_event;        // 初始化中传入的事件（通常是外设事件）
    uint32_t compWaitValue = 0;
    uint32_t isCompWait = xTaskGenericNotifyWait(NotifyCompletionIndex, 0, compWaitEvent, &compWaitValue, timeout);
    if(isCompWait && (compWaitValue & compWaitEvent))
        return AWLF_OK;         // 被通知唤醒
    else
        return AWLF_ERROR_TIMEOUT;      // 超时唤醒
}

/* 对等待方进行恢复、通知等操作，严禁阻塞 */
static awlf_ret_t CompletionDone(completion_t c)
{
    long isNeedSwitch = 0;
    if(xPortIsInsideInterrupt())
    {
        xTaskGenericNotifyFromISR(c->wait_thread, NotifyCompletionIndex, c->comp_event, eSetBits, NULL, &isNeedSwitch);
        if(isNeedSwitch)
            portYIELD_FROM_ISR(isNeedSwitch);
    }
    else
        xTaskGenericNotify(c->wait_thread, NotifyCompletionIndex, c->comp_event, eSetBits, NULL);
    return AWLF_OK;
}

awlf_ret_t completion_wait(completion_t c, size_t timeout)
{
    uint32_t      primask;
    TaskHandle_t  current_thread;
    /* 1. 参数检查 */
    if(!c)
        return AWLF_ERROR_PARAM;
    if(xPortIsInsideInterrupt())
        while(1) {
            // 禁止在中断中等待！！
        }
    
    /* 2. 获取当前线程和关中断 */
    current_thread = xTaskGetCurrentTaskHandle();
    primask = awlf_hw_disable_irq();
    if(c->is_done != COMP_DONE)
    {
        if(c->wait_thread != NULL)
            while(1) {
                // 同一时间同一个完成量仅能被一个线程等待！！有一对多需求请使用其他IPC对象
            }
        c->wait_thread = current_thread;
        c->is_done = COMP_WAIT;
        awlf_hw_enable_irq(primask);
        if(CompletionWait(c, timeout) != AWLF_OK)
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
    awlf_hw_enable_irq(primask);
    return AWLF_OK;
}

awlf_ret_t completion_done(completion_t c)
{
    uint32_t   primask;
    awlf_ret_t ret;
    if(!c) return AWLF_ERROR_PARAM;
    primask = awlf_hw_disable_irq();

    /* 存在一种情况：多次通知，但是等待方未来得及响应，此时不重复操作 */
    if(c->is_done == COMP_DONE)
    {
        awlf_hw_enable_irq(primask);
        return AWLF_ERROR_BUSY;
    }
    
    TaskHandle_t task = c->wait_thread;
    if(task) 
    {
        if(CompletionDone(c) != AWLF_OK)
        {
            awlf_hw_enable_irq(primask);
            ret = AWLF_ERROR_EMPTY;
        }
        c->wait_thread = NULL;
        ret = AWLF_OK;
    }
    else
    {
        awlf_hw_enable_irq(primask);
        ret = AWLF_ERROR_EMPTY;
    }
    c->is_done = COMP_DONE;
    awlf_hw_enable_irq(primask);
    return ret;
}

void completion_init(completion_t c, uint32_t event) 
{
    uint32_t   primask;
    primask = awlf_hw_disable_irq();
    c->is_done     = COMP_INIT;
    c->comp_event = event;
    awlf_hw_enable_irq(primask);
}


