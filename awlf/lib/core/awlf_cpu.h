/**
 * @file  awlf_board.h
 * @brief 板级函数接口
 * @note  1、本文件规范了不同开发板的行为，以便于工程迁移
 *        2、本文件作为一个规范，仅包含函数声明，具体实现请浏览platform/board/awlf_board.c
 */

#ifndef __AWLF_BOARD_H__
#define __AWLF_BOARD_H__

#include "awlf_config.h"
#include "awlf_def.h"
#include <stdint.h>

/* CPU 配置 */
#define __AWLF_CPU_NAME "DJI_C" // CPU名称，如"DJI_C"
typedef uint32_t cputime_cnt;   // CPU时间计数器类型，由此计数类型，通过换算关系计算得到以下时间类型
typedef float    cputime_s;     // CPU时间类型，单位：s
typedef float    cputime_ms;    // CPU时间类型，单位：ms
typedef uint64_t cputime_us;    // CPU时间类型，单位：us

typedef struct AwlfCpu* AwlfCpu_t;

typedef struct AwlfBoardInterface* AwlfBoardInterface_t;
typedef struct AwlfBoardInterface
{
    void (*errhandler)(void);
    void (*reset)(void);
    cputime_s (*get_cpu_time_s)(void);                           // 获取CPU时间，单位：s
    cputime_ms (*get_cpu_time_ms)(void);                         // 获取CPU时间，单位：ms
    cputime_us (*get_cpu_time_us)(void);                         // 获取CPU时间，单位：us
    cputime_s (*get_delta_cpu_time_s)(cputime_cnt* lastTimeCnt); // 获取CPU时间差，单位：s
    void (*delay_ms)(float ms);                                  // 延时ms
} AwlfBoardInterface_s;

typedef struct AwlfCpu
{
    char*                cpuName;     // CPU名称，如"DJI_C"
    char*                awlfVersion; // AWLF版本，目前采用时间制，如"2025-12-1"
    cputime_cnt          lastTimeCnt; // 上一次获取的CPU时间计数器值
    uint32_t             cpuFreqMHz;  // CPU频率，单位：MHz
    uint32_t             cpuFreqHz;   // CPU频率，单位：Hz
    AwlfBoardInterface_t interface;
} AwlfCpu_s;

/**
 * @brief 获取CPU名称
 *
 * @return char* CPU名称
 */
char* awlf_get_cpu_name(void);

/**
 * @brief 获取固件版本
 *
 * @return char* 固件版本
 */
char* awlf_get_firmware_version(void);

/**
 * @brief 获取CPU时间
 *
 * @return cputime_t CPU时间，单位：s
 */
cputime_s awlf_cpu_get_time_s(void);

/**
 * @brief 获取CPU时间
 *
 * @return cputime_ms CPU时间，单位：ms
 */
cputime_ms awlf_cpu_get_time_ms(void);

/**
 * @brief 获取CPU时间
 *
 * @return cputime_us CPU时间，单位：us
 */
cputime_us awlf_cpu_get_time_us(void);

/**
 * @brief 获取CPU时间差
 *
 * @return cputime_s CPU时间差，单位：s
 */
cputime_s awlf_cpu_get_delta_time_s(cputime_cnt* lastTimeCnt);

/**
 * @brief 处理CPU错误
 *
 * @param file 错误发生的文件名
 * @param line 错误发生的行号
 * @param msg 错误信息
 */
void awlf_cpu_errhandler(char* file, uint32_t line, uint8_t level, char* msg);

#define AWLF_CPU_ERRHANDLER(msg, level) awlf_cpu_errhandler(__FILE__, __LINE__, level, msg)

/**
 * @brief CPU复位
 *
 */
void awlf_cpu_reset(void);

/**
 * @brief 延时ms
 *
 * @param ms 延时时间，单位：ms
 */
void awlf_cpu_delay_ms(float ms);

/**
 * @brief 注册CPU
 *
 * @param cpuFreqMHz CPU频率，单位：MHz
 * @param interface 板级接口
 */
void awlf_cpu_register(uint32_t cpuFreqMHz, AwlfBoardInterface_t interface);

/**
 * @brief 初始化CPU
 *
 */
void awlf_cpu_init(void);

/**
 * @brief 初始化板级函数
 *
 * @note  1、本函数用于初始化板级函数，用于板级初始化，以及注册cpu
 *        2、本函数为弱函数，可在底层重写
 */
void awlf_bsp_init(void);

#endif
