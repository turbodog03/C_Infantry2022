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
    //TODO:�ҵ��°�������ĵط����Ҹ��ط������������������
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
//TODO����Ҫ���Ķ��������̻����źš�gim.control_mode��yaw_relative_angle
//TODO: ?Ӧ�������ڽ��յı�����volatile�ɣ����շ��Ǳ߼ӣ��������ͷ����Ҳ�ᱻ����̸߳ı䰡�������������ᱻgimbal���̸߳ı�
void get_communicate_info(uint8_t* data){
    //c��֧�ָ�������λ��������Ϊ������������������治ͬ
    //���ｫ������ת��Ϊ������������λ(����ǿ������ת�����ǽ��������ռ���ڴ�Ľ��͹���ת��Ϊ����
    uint32_t *yaw_angle = (uint32_t *)&yaw_relative_angle;
    //��һλ�����ж��Ƿ��ѵ���
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