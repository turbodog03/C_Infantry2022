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
    //TODO:�ҵ��°�������ĵط����Ҹ��ط������������������
    for(;;) {

        Send_RC_Data(&hcan2,sbus_rx_buf[0]);
        Send_RC_Data(&hcan2,sbus_rx_buf[1]);
        get_communicate_info(cm_data);
        write_can(hcan2,CAN_UP_TX_INFO,cm_data);
        osDelayUntil(&transmission_wake_time, 50);
    }
    /* USER CODE END transmit_task */
}
//TODO����Ҫ���Ķ��������̻����źš�gim.control_mode��yaw_relative_angle
//TODO: ?Ӧ�������ڽ��յı�����volatile�ɣ����շ��Ǳ߼ӣ��������ͷ����Ҳ�ᱻ����̸߳ı䰡�������������ᱻgimbal���̸߳ı�
void get_communicate_info(uint8_t* cm_data){
    //��һλ�����ж��Ƿ��ѵ���
    cm_data[0] = enable_chassis;
    cm_data[1] = gim.ctrl_mode;
    //TODO:����ת��֮������λû�����𣿵��ǲ�����ת����û����λ��
    cm_data[2] = (uint32_t)yaw_relative_angle >> 24;
    cm_data[3] = (uint32_t)yaw_relative_angle >> 16;
    cm_data[4] = (uint32_t)yaw_relative_angle >> 8;
    cm_data[5] = (uint32_t)yaw_relative_angle;
    cm_data[6] = stop_chassis;
}
