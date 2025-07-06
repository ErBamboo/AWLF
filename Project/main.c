#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "awlf_device.h"
#include "bsp_serial.h"

void serial_rx_done_callback(device_t dev, void* itembuf, size_t itemsz)
{
    device_write(dev, 0, itembuf, itemsz);
}

void SerialTestTask(void *pvParameters) 
{
    uint8_t data_buf[500] = {1,2,3,4,5,6,7,8,9,10};
    uint8_t recv_buf[100] = {0};
    size_t recv_len = 0;
    device_t serial = device_find("usart1");
    device_open(serial, OTYPE_BLOCKING_TX | OTYPE_BLOCKING_RX);
    
    while(1)
    {
        recv_len = device_read(serial, 0, data_buf, 460);
        if(recv_len)
        {
            device_write(serial, 0, data_buf, 460);
        }
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