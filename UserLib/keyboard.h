//
// Created by turboDog on 2021/11/17.
//

#ifndef BOARD_C_INFANTRY_KEYBOARD_H
#define BOARD_C_INFANTRY_KEYBOARD_H
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


#include "stm32f4xx_hal.h"

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/

/**
  * @brief     �����˶��ٶȿ���ģʽ
  */
typedef enum
{
    NORMAL_MODE = 0,    //����ģʽ
    FAST_MODE,          //����ģʽ
    SLOW_MODE,          //����ģʽ
} kb_move_e;

/**
  * @brief     ��갴��״̬����ö��
  */
typedef enum
{
    KEY_RELEASE = 0,    //û�а�������
    KEY_WAIT_EFFECTIVE, //�ȴ�����������Ч������
    KEY_PRESS_ONCE,     //��������һ�ε�״̬
    KEY_PRESS_DOWN,     //�����Ѿ�������
    KEY_PRESS_LONG,     //��������״̬
} kb_state_e;

/**
  * @brief     ����������ݽṹ��
  */
typedef struct
{
    /* ����ģʽʹ�ܱ�־ */
    uint8_t kb_enable;

    /* �����̿���ģʽ�µĵ����ƶ��ٶ�Ŀ��ֵ */
    float vx;          //����ǰ������Ŀ���ٶ�
    float vy;          //��������ƽ��Ŀ���ٶ�
    float vw;          //������ת�ٶ�
    float max_spd;     //�˶�����ٶ�

    /* ���Ұ���״̬ */
    kb_state_e lk_sta; //��ఴ��״̬
    kb_state_e rk_sta; //�Ҳఴ��״̬

    uint16_t lk_cnt;
    uint16_t rk_cnt;

    /* �˶�ģʽ�����̿��Ƶ����˶����� */
    kb_move_e move_mode;

} km_control_t;

extern km_control_t km;

/**
  * @brief     PC ����������ݴ�����
  */
void pc_kb_hook(void);

#endif //BOARD_C_INFANTRY_KEYBOARD_H
