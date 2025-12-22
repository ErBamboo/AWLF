#ifndef __AWLF_PORT_COMPILER_H__
#define __AWLF_PORT_COMPILER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(__ARMCC_VERSION) /* ARM Compiler */
    #define __port_attribute(x) __attribute__(x)
    #define __port_used         __port_attribute((used))
    #define __port_weak         __port_attribute((weak))
    #define __port_section(x)   __port_attribute((section(x)))
    #define __port_align(n)     __port_attribute((aligned(n)))
    #define __port_noreturn     // for C++
    #if __ARMCC_VERSION >= 6010050
        #define __port_packed __attribute__((packed))
    #else
        #define __port_packed __packed
    #endif
#endif

#ifdef __cplusplus
}
#endif

#endif // __AWLF_PORT_COMPILER_H__
