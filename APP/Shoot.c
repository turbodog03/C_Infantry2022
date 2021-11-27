//
// Created by turboDog on 2021/11/21.
//

#include "Shoot.h"
#include "bsp_can.h"
#include "pid.h"
#include "bsp_uart.h"
#include "keyboard.h"
#include "Detect.h"
#include "stdlib.h"
#include "bsp_tim.h"


extern void turn_on_off_friction_wheel(void);
void cap_control(void);
void auto_shoot_control(void);
extern int16_t trigger_moto_speed_ref;
extern int32_t  trigger_moto_position_ref; //�������λ��Ŀ��ֵ
extern char mmp_buf[20];
void shoot_custom_control(void);
int cap_open_flag = 0;
int cap_ok = 0;
int auto_shoot_cmd = 0;
int auto_shoot_ok = 1;


/* ���������ز��� */
enum SHOOT_STATE shoot_state;
uint8_t   shoot_cmd = 0;
uint32_t  continue_shoot_time;
//uint8_t   continuous_shoot_cmd = 0;
uint8_t   fric_wheel_run = 0;
uint16_t  fric_wheel_speed = SHOT_FRIC_WHEEL_SPEED;
/* �ϴε�ң���������� */
uint8_t   last_left_key;
uint8_t   last_right_key;
uint8_t   last_sw1;
uint8_t   last_sw2;
int16_t   last_wheel_value;
//ң��������Ħ����
#define RC_FIRC_CTRL     ((last_sw1 != RC_UP) && (rc.sw1 == RC_UP))           //((last_wheel_value != -660) && (rc.wheel == -660))
//ң��������
#define RC_SINGLE_TRIG   ((last_sw1 != RC_MI) && (rc.sw1 == RC_MI))           // ((last_wheel_value != 660) && (rc.wheel == 660))
//ң��������
#define RC_CONTIN_TRIG   ((rc.sw1 == RC_MI) && (HAL_GetTick() - continue_shoot_time >= 1500))   //((rc.wheel == 660) && (HAL_GetTick() - continue_shoot_time >= 1000))
//ң�����˳�����ģʽ
#define EXIT_CONTIN_TRIG (last_sw1 != RC_MI)                                 // (rc.wheel <= 600)

void shoot_task(const void* argu)
{
    //��������ʼ��

    /* �������PID������ʼ�� */
    pid_init(&pid_trigger, 4500, 2000, 0.15f, 0.005, 0);
    pid_init(&pid_trigger_speed, 7000, 3000, 7.0, 0.5, 0.1);
    /* Ħ���ֵ��PID������ʼ�� */
    pid_init(&pid_shoot_left, 7000, 3000, 9.0f, 0.02, 0.00);
    pid_init(&pid_shoot_right, 7000, 3000, 9.0f, 0.02, 0.00);
    uint32_t shoot_wake_time = osKernelSysTick();
    while(1)
    {
        //GPIOE->BSRR=0x1000;
        if (rc.kb.bit.Q && rc.sw2 != RC_DN)
            fric_wheel_run = 1;

        if ((rc.kb.bit.Q && rc.kb.bit.SHIFT) || rc.sw2 == RC_DN)
            fric_wheel_run = 0;

        if (glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist)
            fric_wheel_run = 0;

        //���ص��ָ�
        if (rc.kb.bit.R||rc.sw1!=RC_DN){
            cap_open_flag = -1;
            cap_ok = 0;
        }
        if ((rc.kb.bit.R &&rc.kb.bit.SHIFT) || rc.sw1==RC_DN){
            cap_open_flag = 1;
            cap_ok = 0;
        }

        /* bullet single or continue trigger command control  */
//		GPIOE->ODR|=data_recv.shootCommand<<5;//ʹ��PE5�����cmd
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)data_recv.shootCommand);
        //if ( RC_SINGLE_TRIG ||                 //ң��������
        if((rc.mouse.l||data_recv.shootCommand) && shoot_state==dont_shoot) //������λ������
        {
            data_recv.shootCommand=0;
            shoot_cmd=1;
            continue_shoot_time = HAL_GetTick();
            if(rc.kb.bit.SHIFT)
                shoot_state=trible_shoot;
            else
                shoot_state=single_shoot;
        }
        else if ( RC_CONTIN_TRIG  //ң��������
                  || rc.mouse.r ) 				//�������
        {
            shoot_state=continuous_shoot;
            trigger_moto_position_ref=moto_trigger.total_ecd;
        }
        else if(HAL_GetTick()-continue_shoot_time>600)
        {
            shoot_state=dont_shoot;
        }

        if (fric_wheel_run == 0)
        {
            shoot_state=dont_shoot;
        }


        /* �����������ʵ�ֺ��� */
        shoot_custom_control();
        cap_control();

        /* ����Ħ���ֲ������� */
        if(rc.sw1&&(last_sw1!=rc.sw1))
        {
            last_sw1 = rc.sw1;
            if(rc.sw1==RC_UP)
                fric_wheel_run = !fric_wheel_run;
        }
        /* ����Ħ����ʵ�ֺ��� */
        turn_on_off_friction_wheel();

        last_sw2 = rc.sw2;
        last_left_key    = km.lk_sta;
        last_right_key   = km.rk_sta;
        last_wheel_value = rc.wheel;
        //GPIOE->BSRR=0x10000000;
        osDelayUntil(&shoot_wake_time, SHOOT_PERIOD);
    }
}


void cap_control(){
    if(!cap_ok){
        if(cap_open_flag == 1){
            set_pwm_param(1, 1000);
        }
        else{
            set_pwm_param(1,2000);
        }
        start_pwm_output(1);
        cap_ok = 1;
    }
}


//void auto_shoot_control(){
//	if(data_recv.shootCommand == 1){
//		if(auto_shoot_ok){
//			auto_shoot_cmd =0;
//		}
//		else{
//			auto_shoot_cmd = 1;
//			auto_shoot_ok = 1;
//		}
//	}
//	else{
//		auto_shoot_ok = 0;
//	}
//}


/* �����������λ��(��λ�Ǳ�������ֵencoder) */
int32_t trigger_moto_position_ref;
/* �����������ת��(rpm) */
int16_t trigger_moto_speed_ref;
/* ����������� */
int16_t trigger_moto_current;
/* Ħ���ֵ������ */
int16_t shoot_moto_current_left;
int16_t shoot_moto_current_right;

/* ������������������ */
extern int16_t  trigger_moto_speed_ref;    //��������ٶ�Ŀ��ֵ
extern uint8_t  fric_wheel_run;            //Ħ���ֿ���
extern uint16_t fric_wheel_speed;          //Ħ�����ٶ�
extern int32_t  trigger_moto_position_ref; //�������λ��Ŀ��ֵ


float speedInNumsPerSec;
uint32_t numsOfOneShot;
uint32_t delayTimeInMs;

/* �������� */
uint32_t stall_count = 0;
uint32_t stall_inv_count = 0;
uint8_t  stall_f = 0;
void block_bullet_handle(void)
{
    if (pid_trigger_speed.out <= -5000)  //��������
    {
        if (stall_f == 0)
            stall_count ++;
    }
    else
        stall_count = 0;

    if (stall_count >= 600) //����ʱ��3s
    {
        stall_f = 1;
        stall_count = 0;

    }

    if (stall_f == 1)
    {
        stall_inv_count++;

        if (stall_inv_count >= 100)  //��תʱ��0.5s
        {
            stall_f = 0;
            stall_inv_count = 0;
        }
        else
            trigger_moto_speed_ref = 2000;
    }
}

/**
  * @brief          ����������Ƶ���㲦������ٶȣ���ʵ����������������Ƶ�������������ӵ�
  * @param[1]       ��Ƶ����λ����/s
  * @param[2]       ��������ӵ���
  * @param[3]       ����������ʱ��
  * @retval         ������������ٶ� ��λ��RPM
  */
float ShootAndDelay(float speedInNumsPerSec, uint32_t numsOfOneShot, uint32_t delayTimeInMs)
{
    static uint32_t ticksInMs = 0, lastNumsOfOneShot = 0, lastDelayTimeInMs = 0, count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0 || lastNumsOfOneShot != numsOfOneShot || lastDelayTimeInMs != delayTimeInMs)
    {
        ticksInMs = HAL_GetTick() + delayTimeInMs + 1;
        lastAngle = moto_trigger.total_angle;
    }
    if (lastAngle - moto_trigger.total_angle > 8191 * TRIGGER_MOTOR_REDUCTION_RATIO / BULLETS_PER_ROUND * numsOfOneShot)
    {
        lastAngle = moto_trigger.total_angle;
        speed = 0;
        ticksInMs = HAL_GetTick();
    }

    if (HAL_GetTick() - ticksInMs > delayTimeInMs)
        speed = speedInNumsPerSec / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60;

    count++;
    lastNumsOfOneShot = numsOfOneShot;
    lastDelayTimeInMs = delayTimeInMs;
    return speed;
}

/* �ӵ��ĵ������������� */
void shoot_custom_control(void)
{
    static uint8_t count=0;
    if (fric_wheel_run)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,(GPIO_PinState)shoot_cmd);
        switch(shoot_state)
        {
            case single_shoot:
                if(shoot_cmd)
                {
                    /* ����ǵ������������ת45�� */
                    trigger_moto_position_ref = moto_trigger.total_ecd + DEGREE_45_TO_ENCODER;
                    shoot_cmd=0;
                }
                /* �ջ����㲦���������ת�� */
                trigger_moto_speed_ref = pid_calc(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
                goto emmm;
            case trible_shoot:
                if(shoot_cmd)
                {
                    /* ������������������ת3*45�� */
                    trigger_moto_position_ref = moto_trigger.total_ecd + 3*DEGREE_45_TO_ENCODER;
                    shoot_cmd=0;
                }
                /* �ջ����㲦���������ת�� */
                trigger_moto_speed_ref = pid_calc(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
                goto emmm;
            case continuous_shoot:
                speedInNumsPerSec=4.0f;
                numsOfOneShot=4;
                delayTimeInMs=10;
                break;
            case dont_shoot:
                trigger_moto_speed_ref = 0;
                goto emmm;
        }
        trigger_moto_speed_ref=-ShootAndDelay(speedInNumsPerSec,numsOfOneShot,delayTimeInMs);
        block_bullet_handle();                                 //��������
        /* �ջ����㲦��������� */
        emmm:
        trigger_moto_current = pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, trigger_moto_speed_ref);
    }
    else
    {
        trigger_moto_current = 0;
    }
    /* �ջ�����Ħ���ֵ������ */
//	count%=4;
//	if(!count)
//	{
    shoot_moto_current_left = pid_calc(&pid_shoot_left, moto_shoot[0].speed_rpm, -fric_wheel_speed);
    shoot_moto_current_right = pid_calc(&pid_shoot_right, moto_shoot[1].speed_rpm, fric_wheel_speed);
//	}
    /* ���Ͳ��������Ħ���ֵ������ */
    send_shoot_moto_current(shoot_moto_current_left,shoot_moto_current_right,trigger_moto_current);
//	count++;
}

/* ����Ħ���ִ��� */
void turn_on_off_friction_wheel(void)
{
    if (fric_wheel_run)
    {
        //��Ħ����
        fric_wheel_speed=SHOT_FRIC_WHEEL_SPEED;
        //�򿪼��⡢����װ��
        HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);
        //TODO:���⣬����û�ӣ���ע����
        //write_led_io(LASER_IO, LED_ON);
    }
    else
    {
        //�ر�Ħ����
        fric_wheel_speed=0;
        //�رռ��⡢����װ��
        HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);
        //write_led_io(LASER_IO, LED_OFF);
    }
}


