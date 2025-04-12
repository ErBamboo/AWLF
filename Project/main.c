#include "stm32f4xx.h"
#include "awlf_include.h"
#include "serial.h"
#include "bsp_serial.h"

completion_s serial_rx_comp;
void serial_rx_done_callback(f_device_t dev, void* itembuf, size_t itemsz)
{
    device_write(dev, 0, itembuf, itemsz);
}

void SerialTestTask(void *pvParameters) 
{
    TaskHandle_t task2_self = f_task_get_current_handle();
    uint8_t data_buf[500] = {1,2,3,4,5,6,7,8,9,10};
    uint8_t recv_buf[100] = {0};
    size_t recv_len = 0;
    f_device_t serial = device_find("usart1");
    device_open(serial, OTYPE_BLOCKING_TX | OTYPE_BLOCKING_RX);
    // device_set_rx_callback(serial, NULL);
    // device_init(serial);
    while(1)
    {
        recv_len = device_read(serial, 0, data_buf,460);
        if(recv_len)
        {
            device_write(serial, 0, data_buf, 460);
        }
    }
}

port_task_handle_t port_task2_handle;
port_task_handle_t port_task1_handle;

int main(void) 
{

    f_task_s task1, task2;
    awlf_ret_t result1, result2;

    result2 = f_task_create(&task2, &port_task2_handle, "SerialTestTask", 4, 4096, SerialTestTask, NULL);
    if (result2 != AWLF_OK) {
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