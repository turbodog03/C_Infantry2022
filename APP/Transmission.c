//
// Created by HLiamso on 2021-11-14.
//


#include "Transmission.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "Gimbal.h"
#include "QuaternionAHRS.h"

//
//uint8_t down_info[8];
static uint8_t cm_data[8] = {0};
char vofatail[4] = {0x00, 0x00, 0x80, 0x7f};
void transmission_task(void const * argument)
{
    /* USER CODE BEGIN transmit_task */
    uint32_t transmission_wake_time = osKernelSysTick();
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
    float angle[3]={0};
    /* Infinite loop */
    //TODO:找到下板程序挂起的地方，找个地方接收这个变量唤醒它
    for(;;) {

        Send_RC_Data(&COM_CAN,sbus_rx_buf[0]);
        osDelay(1);
        Send_RC_Data(&COM_CAN,sbus_rx_buf[1]);
        get_communicate_info(cm_data);
        //write_can(COM_CAN,CAN_UP_TX_INFO,cm_data);
        osDelay(1);
        Send_CM_Data(&COM_CAN,cm_data);
        angle[0]=-AHRS.Pitch;
        angle[1]=-AHRS.Roll;
        angle[2]=-AHRS.Yaw;
        write_uart(1,(uint8_t *)&angle,sizeof(float)*3);
        write_uart(1, (uint8_t *)&vofatail, sizeof(vofatail));
        osDelayUntil(&transmission_wake_time, 45);
    }
    /* USER CODE END transmit_task */
}
//TODO：需要传的东西：底盘唤醒信号、gim.control_mode、yaw_relative_angle
//TODO: ?应该是用于接收的变量加volatile吧？接收方那边加？不过发送方这边也会被别的线程改变啊，比如这三个会被gimbal的线程改变
void get_communicate_info(uint8_t* data){
    //c不支持浮点数移位操作，因为浮点数储存和整数储存不同
    //这里将浮点数转化为整型数进行移位(不是强制类型转换，是将这个储存空间的内存的解释规则转化为整形
    uint32_t *yaw_angle = (uint32_t *)&yaw_relative_angle;
    //第一位用于判断是否唤醒底盘
    data[0] = enable_chassis;
    data[1] = gim.ctrl_mode;
    data[2] = *yaw_angle >> 24;
    data[3] = *yaw_angle >> 16;
    data[4] = *yaw_angle >> 8;
    data[5] = *yaw_angle;
    data[6] = stop_chassis;
}

void Send_CM_Data(CAN_HandleTypeDef *_hcan, uint8_t *data) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_UP_TX_INFO;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    TX_MSG.TransmitGlobalTime = DISABLE;
    CAN_Send_Data[0] = data[0];
    CAN_Send_Data[1] = data[1];
    CAN_Send_Data[2] = data[2];
    CAN_Send_Data[3] = data[3];
    CAN_Send_Data[4] = data[4];
    CAN_Send_Data[5] = data[5];
    CAN_Send_Data[6] = data[6];
    CAN_Send_Data[7] = data[7];
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}