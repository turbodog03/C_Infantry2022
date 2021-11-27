//
// Created by turboDog on 2021/11/21.
//

#ifndef BOARD_C_INFANTRY_BSP_TIM_H
#define BOARD_C_INFANTRY_BSP_TIM_H

//PWM IO �ӿڶ���
//��ʱֻʵ����tim1
#define PWM_GROUP1        1   //��һ�� PWM IO
#define PWM_IO1           1
#define PWM_IO2           2
#define PWM_IO3           3
#define PWM_IO4           4

//#define PWM_GROUP2        2   //�ڶ��� PWM IO
//#define PWM_IO5           5
//#define PWM_IO6           6
//#define PWM_IO7           7
//#define PWM_IO8           8
//
//#define PWM_GROUP3        3   //������ PWM IO
//#define PWM_IO9           9
//#define PWM_IO10          10
//#define PWM_IO11          11
//#define PWM_IO12          12
//
//#define PWM_GROUP4        4   //������ PWM IO
//#define PWM_IO13          13
//#define PWM_IO14          14
//#define PWM_IO15          15
//#define PWM_IO16          16
//PWM IO ��غ���
/**
  * @brief     ���� PWM ���
  * @param     pwm_id: PWM IO �� ID
  */
void start_pwm_output(uint8_t pwm_id);
/**
  * @brief     ���� PWM ���Ӧ����
  * @param     pwm_group: PWM ���Ŀǰ�� 4 ��PWM
  * @param     period:  ÿ�� PWM ��Ӧ����ʱ�䣬��λ��΢��(us)
  */
void set_pwm_group_param(uint8_t pwm_group, uint32_t period);
/**
  * @brief     ���� PWM IO ����
  * @param     pwm_id: PWM IO �� ID
  * @param     pulse: ���� PWM �ĸߵ�ƽʱ�䣬��λ��΢��(us)
  */
void set_pwm_param(uint8_t pwm_id, uint32_t pulse);


#endif //BOARD_C_INFANTRY_BSP_TIM_H
