//
// Created by turboDog on 2021/11/21.
//

#include "ramp.h"

/* yaw ����̨����б�� */
ramp_t yaw_ramp = RAMP_GEN_DAFAULT;
/* pitch ����̨����б�� */
ramp_t pit_ramp = RAMP_GEN_DAFAULT;

/**
  * @brief     б�¿��ƽṹ���ʼ��
  * @param[in] ramp: б�����ݽṹ��ָ��
  * @param[in] scale: �������ݱ仯б��
  */
void ramp_init(ramp_t *ramp, int32_t scale)
{
    ramp->count = 0;
    ramp->scale = scale;
}
/**
  * @brief     б�¿��Ƽ��㺯��
  * @param[in] ramp: б�����ݽṹ��ָ��
  * @retval    б�¿��Ƽ������
  */
float ramp_calc(ramp_t *ramp)
{
    if (ramp->scale <= 0)
        return 0;

    if (ramp->count++ >= ramp->scale)
        ramp->count = ramp->scale;

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}
