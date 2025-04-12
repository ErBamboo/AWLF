#ifndef __F_API_H__
#define __F_API_H__

#include "f_port.h"
#include "f_def.h"

/* 线程同步相关api */

/* frame hw irq ctrl */
#define hw_enable_irq()   port_enable_irq()
#define hw_disable_irq()  port_disable_irq()
#define hw_get_primask()  port_get_primask()
#define hw_set_primask(x) port_set_primask(x)

/* os ctrl api */
#define f_enter_critical()      portEnterCriticalFunc()
#define f_exit_critical()       portExitCriticalFunc()

/* 内存屏蔽 */
#define f_smp_wmb()             port_smp_wmb()

#define f_task_delay_ms(x)          portDelayFunc()
#define f_task_delay_until(x)       portDelayUntilFunc()
/* hw irq ctrl api */
#define f_hw_disable_irq() ({   \
    uint32_t primask = hw_get_primask();  \
    hw_disable_irq(); \
    primask;    \
})

#define f_hw_enable_irq(x) ({   \
    hw_set_primask(x); \
    hw_enable_irq(); \
})

/* atomic bit ops */
#define __SET_FLAG(group, bits)         __atomic_fetch_or(&(group), (bits), __ATOMIC_RELEASE)
#define __IS_FLAG_SET(group, bits)    (__atomic_load_n(&(group), __ATOMIC_ACQUIRE) & (bits))
#define __CLR_FLAG(group, bits) \
do{ \
    uint32_t _bits = ~bits;     \
    __atomic_fetch_and(&(group), _bits, __ATOMIC_RELEASE); \
}while(0)   

#endif
