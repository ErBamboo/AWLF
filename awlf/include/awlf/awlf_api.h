#ifndef __AWLF_API_H__
#define __AWLF_API_H__

#include "port/awlf_port_hw.h"          /* 硬件相关API */
#include "port/awlf_port_compiler.h"   /* 编译器相关API */

/* 硬件中断 */
#define hw_enable_irq()         port_enable_irq()
#define hw_disable_irq()        port_disable_irq()
#define hw_get_primask()        port_get_primask()
#define hw_set_primask(x)       port_set_primask(x)

/* 硬件中断控制 */
#define awlf_hw_disable_irq() ({   \
    uint32_t primask = hw_get_primask();  \
    if(!primask)        \
        hw_set_primask(1); \
    primask;    \
})
#define awlf_hw_enable_irq(x) ({   \
    if(!x)           \
        hw_set_primask(0); \
})

/* 原子操作 */
#define awlf_atomic_fetch_or(ptr, value)    port_atomic_fetch_or(ptr, value)
#define awlf_atomic_load(ptr)               port_atomic_load(ptr)               
#define awlf_atomic_fetch_and(ptr, value)   port_atomic_fetch_and(ptr, value)   
#define awlf_atomic_store(ptr, value)       port_atomic_store(ptr, value)      
#define awlf_atomic_fetch_add(ptr, value)   port_atomic_fetch_add(ptr, value)   
#define awlf_atomic_compare_exchange(ptr, expected, desired) port_atomic_compare_exchange(ptr, expected, desired)
#define awlf_atomic_exchange(ptr, value)    port_atomic_exchange(ptr, value)   
#define awlf_atomic_fetch_sub(ptr, value)   port_atomic_fetch_sub(ptr, value)   

/* 原子位操作，仅适用于32位 */
#define atomic_test_and_set(ptr, value) ({ \
    uint32_t old_value = port_atomic_load(ptr); \
    port_atomic_store(ptr, old_value | (value)); \
    (old_value & (value)) != 0; \
})
#define atomic_test_and_clear(ptr, value) ({ \
    uint32_t old_value = port_atomic_load(ptr); \
    port_atomic_store(ptr, old_value & ~(value)); \
    (old_value & (value)) != 0; \
})
#define atomic_test_and_set_bit(ptr, bit) atomic_test_and_set(ptr, (1 << bit))
#define atomic_test_and_clear_bit(ptr, bit) atomic_test_and_clear(ptr, (1 << bit))

/* 原子标志操作 */
#define __SET_FLAG(group, bits)        port_atomic_fetch_or(&(group), (bits))
#define __IS_FLAG_SET(group, bits)    (port_atomic_load(&(group)) & (bits))
#define __CLR_FLAG(group, bits) \
do{ \
    uint32_t _bits = ~bits;     \
    port_atomic_fetch_and(&(group), _bits); \
}while(0)   

/* compiler specific */
#define __awlf_attribute(x)             __port_attribute(x)
#define __awlf_used                     __port_used
#define __awlf_weak                     __port_weak
#define __awlf_section(x)               __port_section(x)
#define __awlf_align(n)                 __port_align(n)
#define __awlf_noreturn                 __port_noreturn   // for c++
#define __awlf_packed                   __port_packed

#endif /* __AWLF_API_H__ */