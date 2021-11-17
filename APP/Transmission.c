//
// Created by HLiamso on 2021-11-14.
//


#include "Transmission.h"
#include "bsp_can.h"

void transmission_task(void const * argument)
{
    /* USER CODE BEGIN transmit_task */
    uint32_t transmission_wake_time = osKernelSysTick();
    uint8_t send_data[8];
    send_data[0]= 20;
    send_data[1]= 20;
    send_data[2]= 20;
    send_data[3]= 20;
    send_data[4]= 20;
    send_data[5]= 20;
    send_data[6]= 20;
    send_data[7]= 20;
    /* Infinite loop */
    for(;;) {
        HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
        HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
        //write_can(hcan1,0x200,send_data);
        HAL_Delay(500);
        osDelayUntil(&transmission_wake_time, 500);
    }
    /* USER CODE END transmit_task */
}
