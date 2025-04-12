#ifndef __CORE_PORT_H__
#define __CORE_PORT_H__

/* hw port files 根据实际情况替换 */
#include "stm32f4xx.h"
#include "f_config.h"

#ifdef __GNUC__
#define port_enable_irq()   __enable_irq()
#define port_disable_irq()  __disable_irq()
#define port_get_primask()  __get_PRIMASK()
#define port_set_primask(x) __set_PRIMASK(x)

#define port_smp_wmb()      __sync_synchronize()      // 内存屏蔽函数（其他编译器需要做对应的移植）

#endif /* __GNUC__ */



#if defined(USED_FREERTOS)
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#define portWAIT_TIME_FOREVER     portMAX_DELAY
#define portEnterCriticalFunc()   taskENTER_CRITICAL()
#define portExitCriticalFunc()    taskEXIT_CRITICAL()

#define portCheckIsInISR()              xPortIsInsideInterrupt() // 判断是否在中断中
#define portGetTaskSelf()               xTaskGetCurrentTaskHandle() // 获取当前任务句柄
#define portGetTaskSelfFromName(name)   xTaskGetHandle(name) // 通过任务名获取任务句柄


#define portCreateTaskFunc        xTaskCreate      // 创建任务
#define portDeleteTaskFunc        vTaskDelete      // 删除任务
#define portSuspendTaskFunc       vTaskSuspend     // 挂起任务
#define portResumeTaskFunc        vTaskResume      // 恢复任务
#define portYieldFunc             taskYIELD        // 任务让步
#define portDelayFunc             vTaskDelay       // 任务延时
#define portGetTaskNameFunc       pcTaskGetName    // 获取任务名
#define portGetTaskStateFunc      eTaskGetState    // 获取任务状态
#define portGetTaskPriorityFunc   uxTaskPriorityGet // 获取任务优先级
#define portSetTaskPriorityFunc vTaskPrioritySet         // 设置任务优先级
#define portDelayUntilFunc      vTaskDelayUntil    // 任务延时到指定时间
#define portStartSchedulerFunc  vTaskStartScheduler // 启动调度器
#define portEndSchedulerFunc    vTaskEndScheduler   // 结束调度器
#define portGetTickCountFunc    xTaskGetTickCount   // 获取系统时钟
#define portGetTickRateHzFunc   configTICK_RATE_HZ  // 获取系统时钟频率
#define portGetTaskStackHighWaterMarkFunc uxTaskGetStackHighWaterMark // 获取任务栈高水位
#endif /* USED_FREERTOS */

#endif /* __CORE_PORT_H__ */
