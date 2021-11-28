//
// Created by HLiamso on 2021-11-14.
//


#include "Transmission.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "Gimbal.h"

uint8_t down_info[8];
uint8_t cm_data[8] = {0};
void transmission_task(void const * argument)
{
    /* USER CODE BEGIN transmit_task */
    uint32_t transmission_wake_time = osKernelSysTick();

    /* Infinite loop */
    //TODO:找到下板程序挂起的地方，找个地方接收这个变量唤醒它
    for(;;) {

        Send_RC_Data(&hcan2,sbus_rx_buf[0]);
        Send_RC_Data(&hcan2,sbus_rx_buf[1]);
        get_communicate_info(cm_data);
        write_can(hcan2,CAN_UP_TX_INFO,cm_data);
        osDelayUntil(&transmission_wake_time, 50);
    }
    /* USER CODE END transmit_task */
}
//TODO：需要传的东西：底盘唤醒信号、gim.control_mode、yaw_relative_angle
//TODO: ?应该是用于接收的变量加volatile吧？接收方那边加？不过发送方这边也会被别的线程改变啊，比如这三个会被gimbal的线程改变
void get_communicate_info(uint8_t* cm_data){
    //第一位用于判断是否唤醒底盘
    cm_data[0] = enable_chassis;
    cm_data[1] = gim.ctrl_mode;
    //TODO:类型转换之后再移位没问题吗？但是不类型转换就没法移位？
    cm_data[2] = (uint32_t)yaw_relative_angle >> 24;
    cm_data[3] = (uint32_t)yaw_relative_angle >> 16;
    cm_data[4] = (uint32_t)yaw_relative_angle >> 8;
    cm_data[5] = (uint32_t)yaw_relative_angle;
    cm_data[6] = stop_chassis;
}
