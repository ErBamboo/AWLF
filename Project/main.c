#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "awlf/awlf_api.h"
#include "hal/awlf_hal.h"
#include "bsp_serial.h"

size_t  recv_len = 0;
TaskHandle_t serial_task_handle;
void serial_rd_cb(device_t dev, void* param, size_t paramsz)
{
    
}

void SerialTestTask(void *pvParameters)
{
    uint16_t rx_len = 0;
    uint8_t data_buf[500] = {1,2,3,4,5,6,7,8,9,10};
    device_t serial = device_find("usart1");
    device_open(serial, OPARAM_NON_BLOCKING_TX | OPARAM_BLOCKING_RX);
    device_set_rd_cb(serial, serial_rd_cb);
    
    while(1)
    {
        // rx_len = device_read(serial, NULL, data_buf, 1);
        // if(rx_len)
        // {
        //     device_write(serial, NULL, data_buf, rx_len);
        // }
    }
}

int main(void)
{

    TaskHandle_t task1;
    BaseType_t result1;

    result1 = xTaskCreate(SerialTestTask, "SerialTestTask", 4096, NULL, 4, &task1);
    if (result1 == NULL) {
        while(1) {

        }
    }

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    bsp_serial_init();

    vTaskStartScheduler();

    while(1)
    {

    }
    return 0;
}