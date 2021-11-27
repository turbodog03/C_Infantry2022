//
// Created by turboDog on 2021/11/21.
//

#ifndef BOARD_C_INFANTRY_RAMP_H
#define BOARD_C_INFANTRY_RAMP_H

#include "stm32f4xx_hal.h"

typedef struct ramp_t
{
    int32_t count;
    int32_t scale;
    float   out;
    void  (*init)(struct ramp_t *ramp, int32_t scale);
    float (*calc)(struct ramp_t *ramp);
}ramp_t;

#define RAMP_GEN_DAFAULT \
{ \
  .count = 0, \
  .scale = 0, \
  .out = 0, \
  .init = &ramp_init, \
  .calc = &ramp_calc, \
} \
/**
  * @brief     б�¿��ƽṹ���ʼ��
  * @param[in] ramp: б�����ݽṹ��ָ��
  * @param[in] scale: �������ݱ仯б��
  */
void  ramp_init(ramp_t *ramp, int32_t scale);
/**
  * @brief     б�¿��Ƽ��㺯��
  * @param[in] ramp: б�����ݽṹ��ָ��
  * @retval    б�¿��Ƽ������
  */
float ramp_calc(ramp_t *ramp);

/* yaw ����̨����б�� */
extern ramp_t yaw_ramp;
/* pitch ����̨����б�� */
extern ramp_t pit_ramp;
#endif //BOARD_C_INFANTRY_RAMP_H
