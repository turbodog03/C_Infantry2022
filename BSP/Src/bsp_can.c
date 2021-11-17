//
// Created by HLiamso on 2021-11-14.
//

#include <can.h>
#include <Detect.h>
#include "bsp_can.h"
void CAN_Device_Init(void){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    // CAN1 过滤器初始化
    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
    }
    // 启动CAN1
    while (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
    }
    // 启动通知
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {

    }

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    // CAN2 过滤器初始化
    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
    }
    // 启动CAN2
    while (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
    }
    // 启动通知
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];
//    static uint8_t RC_Data_Buf[16];
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//    if(hcan==&hcan2){
//        switch (rx_header.StdId){
//            case CAN_RC_DATA_Frame_0:
//                RC_Data_Buf[0] = rx_data[0];
//                RC_Data_Buf[1] = rx_data[1];
//                RC_Data_Buf[2] = rx_data[2];
//                RC_Data_Buf[3] = rx_data[3];
//                RC_Data_Buf[4] = rx_data[4];
//                RC_Data_Buf[5] = rx_data[5];
//                RC_Data_Buf[6] = rx_data[6];
//                RC_Data_Buf[7] = rx_data[7];
//                break;
//            case CAN_RC_DATA_Frame_1:
//                RC_Data_Buf[8] = rx_data[0];
//                RC_Data_Buf[9] = rx_data[1];
//                RC_Data_Buf[10] = rx_data[2];
//                RC_Data_Buf[11] = rx_data[3];
//                RC_Data_Buf[12] = rx_data[4];
//                RC_Data_Buf[13] = rx_data[5];
//                RC_Data_Buf[14] = rx_data[6];
//                RC_Data_Buf[15] = rx_data[7];
//
//                break;
//        }
//    }
//    if(hcan==&hcan1){
//    switch (rx_header.StdId){
//        case CAN_3508_M1_ID:
//        {
//            ChassisMotor[0].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[0], rx_data) : \
//            get_moto_info(&ChassisMotor[0], rx_data);
//            err_detector_hook(CHASSIS_M1_OFFLINE);
//        }
//        break;
//        case CAN_3508_M2_ID:
//        {
//            ChassisMotor[1].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[1], rx_data) : \
//            get_moto_info(&ChassisMotor[1], rx_data);
//            err_detector_hook(CHASSIS_M2_OFFLINE);
//        }
//        break;
//
//        case CAN_3508_M3_ID:
//        {
//            ChassisMotor[2].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[2], rx_data) : \
//            get_moto_info(&ChassisMotor[2], rx_data);
//            err_detector_hook(CHASSIS_M3_OFFLINE);
//        }
//        break;
//        case CAN_3508_M4_ID:
//        {
//            ChassisMotor[3].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[3], rx_data) : \
//            get_moto_info(&ChassisMotor[3], rx_data);
//            err_detector_hook(CHASSIS_M4_OFFLINE);
//        }
//        break;
//
//        case CAN_SUPERCAP_RECV:
//            //PowerDataResolve(rx_data);
//        default:
//        {
//        }
//        break;
//        }
//    }
}
void write_can(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]){
    uint32_t send_mail_box;
    tx_message.StdId = send_id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    for(int i=0;i<8;i++)
    can_send_data[i] = send_data[i];
    HAL_CAN_AddTxMessage(&can, &tx_message, can_send_data, &send_mail_box);
}