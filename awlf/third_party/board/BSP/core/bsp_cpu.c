
#include "bsp.h"
#include "core/awlf_interrupt.h"

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
static void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 6;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

static AwlfBoardInterface_s AwlfBoardInterface = {
    .errhandler           = Error_Handler,
    .reset                = HAL_NVIC_SystemReset,
    .get_cpu_time_s       = DWT_GetTimeline_s,
    .get_cpu_time_ms      = DWT_GetTimeline_ms,
    .get_cpu_time_us      = DWT_GetTimeline_us,
    .get_delta_cpu_time_s = DWT_GetDeltaT,
    .delay_ms             = DWT_Delay,
};

void awlf_bsp_init(void)
{
    // 开发板初始化，对于STM32来说，在CubeMX上配置时钟树，然后直接复制过来用就好
    HAL_Init();
    SystemClock_Config();
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // 任务调度前，需要设置中断优先级分组
    // cpu注册
    awlf_cpu_register(__AWLF_CPU_FREQ_MHZ, &AwlfBoardInterface);
    // 外设注册、初始化
    DWT_Init(__AWLF_CPU_FREQ_MHZ);
    bsp_serial_register();
    bsp_can_register();
}
