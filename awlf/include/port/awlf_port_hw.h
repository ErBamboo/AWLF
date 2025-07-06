#ifndef __CORE_PORT_H__
#define __CORE_PORT_H__

/* hw port files 根据实际情况替换 */
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define port_enable_irq()   __enable_irq()
#define port_disable_irq()  __disable_irq()
#define port_get_primask()  __get_PRIMASK()
#define port_set_primask(x) __set_PRIMASK(x)

#define port_atomic_fetch_or(ptr, value)    __atomic_fetch_or((ptr), (value), __ATOMIC_RELEASE)
#define port_atomic_load(ptr)               __atomic_load_n((ptr), __ATOMIC_ACQUIRE)
#define port_atomic_fetch_and(ptr, value)   __atomic_fetch_and((ptr), (value), __ATOMIC_RELEASE)
#define port_atomic_store(ptr, value)       __atomic_store_n((ptr), (value), __ATOMIC_RELEASE)
#define port_atomic_fetch_add(ptr, value)   __atomic_fetch_add((ptr), (value), __ATOMIC_RELEASE)
#define port_atomic_compare_exchange(ptr, expected, desired) __atomic_compare_exchange_n((ptr), (expected), (desired), 0, __ATOMIC_ACQUIRE, __ATOMIC_RELEASE)
#define port_atomic_exchange(ptr, value)    __atomic_exchange_n((ptr), (value), __ATOMIC_RELEASE)
#define port_atomic_fetch_sub(ptr, value)   __atomic_fetch_sub((ptr), (value), __ATOMIC_RELEASE)

#ifdef __cplusplus
}
#endif

#endif /* __CORE_PORT_H__ */
