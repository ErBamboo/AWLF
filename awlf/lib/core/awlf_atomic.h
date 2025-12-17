#ifndef __AWLF_ATOMIC_H__
#define __AWLF_ATOMIC_H__

#include "port/awlf_port_hw.h" /* 硬件相关API */

/* 原子操作 */
#define awlf_atomic_fetch_or(ptr, value)  port_atomic_fetch_or(ptr, value)
#define awlf_atomic_load(ptr)             port_atomic_load(ptr)
#define awlf_atomic_fetch_and(ptr, value) port_atomic_fetch_and(ptr, value)
#define awlf_atomic_store(ptr, value)     port_atomic_store(ptr, value)

/*
    @brief 原子操作 add - 加法 sub - 减法
    @param ptr:     存储地址
    @param value:   加减的值
*/
#define awlf_atomic_fetch_add(ptr, value)                    port_atomic_fetch_add(ptr, value)
#define awlf_atomic_fetch_sub(ptr, value)                    port_atomic_fetch_sub(ptr, value)

#define awlf_atomic_compare_exchange(ptr, expected, desired) port_atomic_compare_exchange(ptr, expected, desired)
#define awlf_atomic_exchange(ptr, value)                     port_atomic_exchange(ptr, value)

/* 原子位操作，仅适用于32位 */
#define atomic_test_and_set(ptr, value)                                                                                          \
    ({                                                                                                                           \
        uint32_t old_value = port_atomic_load(ptr);                                                                              \
        port_atomic_store(ptr, old_value | (value));                                                                             \
        (old_value & (value)) != 0;                                                                                              \
    })
#define atomic_test_and_clear(ptr, value)                                                                                        \
    ({                                                                                                                           \
        uint32_t old_value = port_atomic_load(ptr);                                                                              \
        port_atomic_store(ptr, old_value & ~(value));                                                                            \
        (old_value & (value)) != 0;                                                                                              \
    })
#define atomic_test_and_set_bit(ptr, bit)   atomic_test_and_set(ptr, (1 << bit))
#define atomic_test_and_clear_bit(ptr, bit) atomic_test_and_clear(ptr, (1 << bit))

/* 原子标志操作 */
#define __ATOMIC_SET_FLAG(group, bits)    port_atomic_fetch_or(&(group), (bits))
#define __ATOMIC_IS_FLAG_SET(group, bits) (port_atomic_load(&(group)) & (bits))
#define __ATOMIC_CLR_FLAG(group, bits)                                                                                           \
    do                                                                                                                           \
    {                                                                                                                            \
        uint32_t _bits = ~bits;                                                                                                  \
        port_atomic_fetch_and(&(group), _bits);                                                                                  \
    } while (0)

#endif
