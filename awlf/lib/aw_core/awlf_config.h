#ifndef __AWLF_CONFIG_H__
#define __AWLF_CONFIG_H__

/* 功能裁剪 */
#define __AWLF_USE_POSIX_COMPATIBLE_LAYER // TODO: 使用POXIS兼容层
#define __AWLF_USE_ASSERT                 // TODO: 使能框架断言

/* 抽象层裁剪 */
#define __AWLF_USE_HAL_SERIALS
#define __AWLF_USE_HAL_CAN

#endif // __AWLF_CONFIG_H__