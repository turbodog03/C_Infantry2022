#include "stdint.h"
#include "uart_device.h"
#include "sys.h"
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "usart.h"
#include <string.h>
#include <can.h>

/* 解析后的遥控器数据 */
rc_type_t rc;
/* 接收到的遥控器原始数据 */
uint8_t   dbus_recv[DBUS_FRAME_SIZE];

/**
  * @brief     解析遥控器数据
  * @param     rc: 解析后的遥控器数据结构体指针
  * @param     buff: 串口接收到的遥控器原始数据指针
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff)
{
  /* 下面是正常遥控器数据的处理 */
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  /* 防止遥控器零点有偏差 */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;

  /* 拨杆值获取 */
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  /* 遥控器异常值处理，函数直接返回 */
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_type_t));
    return ;
  }

  /* 鼠标移动速度获取 */
  rc->mouse.x = buff[6] | (buff[7] << 8);
  rc->mouse.y = buff[8] | (buff[9] << 8);
  
  /* 鼠标左右按键键值获取 */
  rc->mouse.l = buff[12];
  rc->mouse.r = buff[13];

  /* 键盘按键键值获取 */
  rc->kb.key_code = buff[14] | buff[15] << 8;
  
  /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
  rc->wheel = buff[16] | buff[17] << 8;
  rc->wheel -= 1024;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        Send_RC_Data(&hcan1, dbus_recv);
        remote_data_handle(&rc, dbus_recv);                        // 遥控器数据解码
        //HAL_UART_Receive_DMA(remote_control.RC_USART, sbus_rx_buf, RC_FRAME_LENGTH); // 开启下一次接收

    }
}

void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_RC_DATA_Frame_0;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = rc_data[0];
    CAN_Send_Data[1] = rc_data[1];
    CAN_Send_Data[2] = rc_data[2];
    CAN_Send_Data[3] = rc_data[3];
    CAN_Send_Data[4] = rc_data[4];
    CAN_Send_Data[5] = rc_data[5];
    CAN_Send_Data[6] = rc_data[6];
    CAN_Send_Data[7] = rc_data[7];
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

    TX_MSG.StdId = CAN_RC_DATA_Frame_1;
    CAN_Send_Data[0] = rc_data[8];
    CAN_Send_Data[1] = rc_data[9];
    CAN_Send_Data[2] = rc_data[10];
    CAN_Send_Data[3] = rc_data[11];
    CAN_Send_Data[4] = rc_data[12];
    CAN_Send_Data[5] = rc_data[13];
    CAN_Send_Data[6] = rc_data[14];
    CAN_Send_Data[7] = rc_data[15];
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}