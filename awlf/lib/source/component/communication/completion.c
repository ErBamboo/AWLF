#include "component/communication/completion.h"
#include "aw_core/awlf_interrupt.h"

/*  1、挂起/阻塞自身，等待被唤醒
    2、简单判断因为超时还是DONE而被唤醒，返回统一错误码
    note: 由于一些设计问题，部分RTOS在进入该函数时，可能需要额外判断Completion的状态
*/
static AwlfRet_e _completion_wait(Completion_t c, size_t timeout)
{
    /* freertos中采用任务通知形式，需要保证configTASK_NOTIFICATION_ARRAY_ENTRIES >= 2，
        因为"通道0"是FREERTOS的默认通道，用于任务间通知。
        为避免冲突，完成量使用"通道1"进行通知 。
        （一个任务至少可以获取32种完成量，绰绰有余）。
    */
    uint32_t compWaitEvent = c->comp_event; // 初始化中传入的事件（通常是外设事件）
    uint32_t compWaitValue = 0;
    uint32_t isCompWait    = xTaskGenericNotifyWait(NotifyCompletionIndex, 0, compWaitEvent, &compWaitValue, timeout);
    if (isCompWait && (compWaitValue & compWaitEvent))
        return AWLF_OK; // 被通知唤醒
    else
        return AWLF_ERROR_TIMEOUT; // 超时唤醒
}

/* 对等待方进行恢复、通知等操作，严禁阻塞 */
static AwlfRet_e _completion_done(Completion_t c)
{
    long isNeedSwitch = 0;
    if (xPortIsInsideInterrupt())
    {
        xTaskGenericNotifyFromISR(c->wait_thread, NotifyCompletionIndex, c->comp_event, eSetBits, NULL, &isNeedSwitch);
        portYIELD_FROM_ISR(isNeedSwitch);
    }
    else
        xTaskGenericNotify(c->wait_thread, NotifyCompletionIndex, c->comp_event, eSetBits, NULL);
    return AWLF_OK;
}

AwlfRet_e completion_wait(Completion_t c, size_t timeout)
{
    uint32_t     primask;
    AwlfRet_e    ret;
    TaskHandle_t current_thread;
    /* 1. 参数检查 */
    if (!c)
        return AWLF_ERROR_PARAM;
    if (xPortIsInsideInterrupt() || c->status == COMP_WAIT)
        while (1)
        {
            // TODO: err log 禁止二次等待或者是在中断中等待！
        }

    /* 2. 获取当前线程和关中断 */
    current_thread = xTaskGetCurrentTaskHandle();
    primask        = awlf_hw_disable_interrupt();
    if (timeout == 0) // 等待0ms，直接返回
        c->status = COMP_DONE;
    switch (c->status)
    {
        case COMP_INIT:
            while (c->wait_thread != NULL) {}; // TODO: assert 存在多个等待线程，请使用其他IPC对象
            c->wait_thread = current_thread;
            c->status      = COMP_WAIT;
            awlf_hw_restore_interrupt(primask);
            // 如果done之后没有立即被唤醒（调度延迟），反而被超时被唤醒，此时需要判断状态来确定是否完成
            if (_completion_wait(c, timeout) != AWLF_OK)
            {
                primask = awlf_hw_disable_interrupt();
                if (c->status != COMP_DONE)
                {
                    ret            = AWLF_ERROR_TIMEOUT;
                    c->wait_thread = NULL;
                    c->status      = COMP_INIT;
                }
                awlf_hw_restore_interrupt(primask);
            }
            else
            {
                ret = AWLF_OK;
            }
            c->status = COMP_INIT;
            break;

        case COMP_WAIT:
            while (1) {}; // TODO: assert 存在多个等待线程
            break;

        case COMP_DONE:
            c->status = COMP_INIT;
            ret       = AWLF_OK;
            break;
    }
    awlf_hw_restore_interrupt(primask);
    return ret;
}

AwlfRet_e completion_done(Completion_t c)
{
    uint32_t  primask;
    AwlfRet_e ret;
    if (!c)
        return AWLF_ERROR_PARAM;
    primask = awlf_hw_disable_interrupt();
    switch (c->status)
    {
        case COMP_INIT: // 若在INIT状态就触发DONE，说明在wait之前就触发了done，此时直接改变状态，避免线程阻塞后等不到完成量释放。
            c->status = COMP_DONE;
            ret       = AWLF_OK;
            break;

        case COMP_WAIT:
            if (c->wait_thread != NULL)
            {
                _completion_done(c);
                c->wait_thread = NULL;
                ret            = AWLF_OK;
                c->status      = COMP_DONE;
            }
            else
            {
                ret = AWLF_ERROR_EMPTY;
            }
            break;

        case COMP_DONE: /* 存在一种情况：多次完成，但是等待方未来得及响应，此时不重复操作 */ ret = AWLF_ERROR_BUSY; break;
    }
    awlf_hw_restore_interrupt(primask);
    return ret;
}

void completion_init(Completion_t c, uint32_t event)
{
    while (!c) {}; // TODO: assert
    c->status     = COMP_INIT;
    c->comp_event = event;
}
