#include "core/awlf_cpu.h"

static AwlfCpu_s AwlfCpu = {
    .cpuName     = __AWLF_CPU_NAME,
    .awlfVersion = __AWLF_VERSION,
};

char* awlf_get_cpu_name(void)
{
    return AwlfCpu.cpuName;
}

char* awlf_get_firmware_version(void)
{
    return AwlfCpu.awlfVersion;
}

cputime_s awlf_cpu_get_time_s(void)
{
    return AwlfCpu.interface->get_cpu_time_s();
}

cputime_ms awlf_cpu_get_time_ms(void)
{
    return AwlfCpu.interface->get_cpu_time_ms();
}

cputime_us awlf_cpu_get_time_us(void)
{
    return AwlfCpu.interface->get_cpu_time_us();
}

cputime_s awlf_cpu_get_delta_time_s(cputime_cnt* lastTimeCnt)
{
    return AwlfCpu.interface->get_delta_cpu_time_s(lastTimeCnt);
}

void awlf_cpu_errhandler(char* file, uint32_t line, uint8_t level, char* msg)
{
    // TODO: 根据level打印不同的日志
    if (level >= AWLF_LOG_LEVEL_MAX)
        level = AWLF_LOG_LEVEL_ERROR;
    if (level == AWLF_LOG_LEVEL_FATAL)
        AwlfCpu.interface->errhandler();
}

void awlf_cpu_reset(void)
{
    // TODO: LOG
    AwlfCpu.interface->reset();
}

void awlf_cpu_delay_ms(float ms)
{
    AwlfCpu.interface->delay_ms(ms);
}

void awlf_cpu_register(uint32_t cpuFreqMHz, AwlfBoardInterface_t interface)
{
    while (!interface) {};
    AwlfCpu.cpuName = __AWLF_CPU_NAME;

    AwlfCpu.awlfVersion = __AWLF_VERSION;
    AwlfCpu.cpuFreqMHz  = cpuFreqMHz;
    AwlfCpu.cpuFreqHz   = cpuFreqMHz * 1000000U;
    AwlfCpu.interface   = interface;
}

__awlf_weak void awlf_bsp_init(void)
{
    while (1) {};
    // 1. 调用awlf_cpu_register注册cpu
    // 2. 板级初始化，如使能时钟树、中断分组等
    // 3. do something else
}

void awlf_cpu_init(void)
{
    awlf_bsp_init();
    while (AwlfCpu.cpuFreqMHz <= 0 || AwlfCpu.cpuFreqHz <= 0) {}; // TODO: assert
    while (AwlfCpu.interface == NULL) {}; // TODO: assert
}
