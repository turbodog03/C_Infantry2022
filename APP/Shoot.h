//
// Created by turboDog on 2021/11/21.
//

#ifndef BOARD_C_INFANTRY_SHOOT_H
#define BOARD_C_INFANTRY_SHOOT_H
#include "sys.h"
#include "cmsis_os.h"
#include "motor.h"

/*************************�����ٶ�����*******************************/
#define SHOT_FRIC_WHEEL_SPEED    7000 //���Ϊ2500
#define SHOT_SUCCESS_FRIC_WHEEL_SPEED  6800  //����ɹ�Ħ���ֻ����
#define SHOT_ABLE_FRIC_WHEEL_SPEED  6000
#define SHOOT_PERIOD 2
/* ���������ı������г� */
#define DEGREE_60_TO_ENCODER  49146
#define DEGREE_45_TO_ENCODER -36864

#define TRIGGER_MOTOR_REDUCTION_RATIO 36
#define BULLETS_PER_ROUND 8

/*************************����Ƶ������*******************************/
#define TRIGGER_MOTOR_SPEED      1500 //

void shoot_task(const void* argu);
void auto_shoot_control(void);
enum SHOOT_STATE
{
    single_shoot,
    trible_shoot,
    continuous_shoot,
    dont_shoot,
};

extern enum SHOOT_STATE shoot_state;
extern uint8_t shoot_cmd;
extern uint8_t fric_wheel_run;
#endif //BOARD_C_INFANTRY_SHOOT_H
