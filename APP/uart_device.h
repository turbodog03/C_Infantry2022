/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 



#ifndef __UART_DEVICE_H__
#define __UART_DEVICE_H__


#include <stm32f4xx_hal.h>

#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132

/**
  * @brief     �������ң�������ݽṹ��
  */
typedef struct 
{
  /* ң������ͨ�����ݣ���ֵ��Χ��-660 ~ 660 */
  int16_t ch1;   //�Ҳ�����
  int16_t ch2;   //�Ҳ�����
  int16_t ch3;   //�������
  int16_t ch4;   //�������
  
  /* ң�����Ĳ������ݣ������·ֱ�Ϊ��1��3��2 */
  uint8_t sw1;   //��ದ��
  uint8_t sw2;   //�Ҳದ��
  
  /* PC ������� */
  struct
  {
    /* ����ƶ���� */
    int16_t x;   //���ƽ��
    int16_t y;   //�������
    /* ��갴����أ�1Ϊ���£�0Ϊ�ɿ� */
    uint8_t l;   //��ఴ��
    uint8_t r;   //�Ҳఴ��
  }mouse;
  
  /* PC ���̰������� */
  union 
  {
    uint16_t key_code;
    struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    }bit;
  }kb;
  
  /* ң������ದ������ */
  int16_t wheel;
} rc_type_t;

/**
  * @brief     ң������������ö��
  */
enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
};


/**
  * @brief     ����ң��������
  * @param     rc: �������ң�������ݽṹ��ָ��
  * @param     buff: ���ڽ��յ���ң����ԭʼ����ָ��
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff);
/**
  * @brief     ң�����жϻص������������� UART ����ʱע��
  */
void dbus_uart_callback(void);
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);

extern rc_type_t rc;
extern uint8_t   dbus_recv[];

#endif
