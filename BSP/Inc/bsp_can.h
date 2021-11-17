//
// Created by HLiamso on 2021-11-14.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <motor.h>
#include "stm32f4xx_hal_def.h"
#endif //BSP_CAN_H
#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132
static CAN_TxHeaderTypeDef  tx_message;
static uint8_t              can_send_data[8];
extern Motor_t ChassisMotor[4];
typedef enum
{
    //接收ID
    CAN_3508_M1_ID       = 0x201,
    CAN_3508_M2_ID       = 0x202,
    CAN_3508_M3_ID       = 0x203,
    CAN_3508_M4_ID       = 0x204,
    CAN_YAW_MOTOR_ID     = 0x205,//ID 1 001
    CAN_PIT_MOTOR_ID     = 0x206,//ID 2 010
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_SUPERCAP_RECV    = 0x211,
    //发送ID
    CAN_CHASSIS_ID       = 0x200,
    CAN_SUPER_CAP_ID      = 0X210,
    CAN_GIMBAL_ID        = 0x1ff,

} can_msg_id_e;
void CAN_Device_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
/**
  * @brief     发送 CAN 数据
  * @param     can_id: CAN 设备 ID，只有 CAN1 或者 CAN2
  * @param     send_id: 发送数据 ID
  * @param     send_data: 发送数据指针，大小为 8 位
  */
void write_can(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]);