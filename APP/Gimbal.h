//
// Created by turboDog on 2021/11/17.
//

#ifndef BOARD_C_INFANTRY_GIMBAL_H
#define BOARD_C_INFANTRY_GIMBAL_H

#include "stm32f4xx_hal.h"
#include "bsp_imu.h"
/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 5
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 1000

#define YAW_MOTOR_MAXOUT 30000
#define PITCH_MOTOR_MAXOUT 30000

// Gimbal attitude define
#define GIMBAL_MAX_DEPRESSION -10
#define GIMBAL_MAX_ELEVATION 30

// Gimbal control define
#define RC_STICK_YAW_RATIO 3
#define RC_STICK_PITCH_RATIO 1.5
#define RC_MOUSE_YAW_RATIO 5
#define RC_MOUSE_PITCH_RATIO -5

#define YAW_V_PID_MAXOUT_M 8000
#define YAW_V_PID_MAXINTEGRAL_M 3000
#define YAW_V_PID_KP_M 300
#define YAW_V_PID_KI_M 20.0
#define YAW_V_PID_KD_M 0.01
#define YAW_V_PID_LPF_M 0.000001
#define YAW_V_PID_D_LPF_M 0.000001

#define YAW_A_PID_MAXOUT_M 320
#define YAW_A_PID_MAXINTEGRAL_M 320
#define YAW_A_PID_KP_M 30
#define YAW_A_PID_KI_M 0.5
#define YAW_A_PID_KD_M 0.001

#define YAW_V_PID_MAXOUT_A 8000
#define YAW_V_PID_MAXINTEGRAL_A 3000
#define YAW_V_PID_KP_A 100
#define YAW_V_PID_KI_A 2.0
#define YAW_V_PID_KD_A 0.2
#define YAW_V_PID_LPF_A 0.000001
#define YAW_V_PID_D_LPF_A 0.000001

#define YAW_A_PID_MAXOUT_A 4000
#define YAW_A_PID_MAXINTEGRAL_A 50
#define YAW_A_PID_KP_A 30
#define YAW_A_PID_KI_A 300
#define YAW_A_PID_KD_A 0.05

#define PITCH_V_PID_MAXOUT 16000
#define PITCH_V_PID_MAXINTEGRAL 2000
#define PITCH_V_PID_KP 16
#define PITCH_V_PID_KI 2.0
#define PITCH_V_PID_KD 0.015
#define PITCH_V_PID_LPF 0.000001
#define PITCH_V_PID_D_LPF 0.08

#define PITCH_A_PID_MAXOUT 3000
#define PITCH_A_PID_MAXINTEGRAL 600
#define PITCH_A_PID_KP 90.0
#define PITCH_A_PID_KI 180
#define PITCH_A_PID_KD 3.5
#define PITCH_A_PID_LPF 0.1
#define PITCH_A_PID_D_LPF 0.01

#define PITCH_V_FFC_MAXOUT 30000
#define PITCH_V_FCC_C0 25
#define PITCH_V_FCC_C1 100 * 0
#define PITCH_V_FCC_C2 0
#define PITCH_V_FCC_LPF 0.005

#define PITCH_A_FFC_MAXOUT 320
#define PITCH_A_FCC_C0 0
#define PITCH_A_FCC_C1 1
#define PITCH_A_FCC_C2 0
#define PITCH_A_FCC_LPF 0.00001

/**
  * @brief     云台控制任务函数
  */
void gimbal_task(const void* argu);

/**
  * @brief     云台控制模式枚举
  */
typedef enum
{
    GIMBAL_INIT = 0,         //云台初始化
    GIMBAL_RELAX = 1,            //云台断电
    GIMBAL_CLOSE_LOOP_ZGYRO = 2, //云台跟随imu z轴角度
    GIMBAL_NO_ACTION = 3,        //无手动信号输入状态
    GIMBAL_AUTO	= 4						 //云台自瞄模式
} gimbal_mode_e;

/**
  * @brief     云台控制信号输入状态枚举
  */
typedef enum
{
    NO_ACTION = 0,           //无控制信号输入
    IS_ACTION,               //有控制信号输入
} action_mode_e;

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
    BACK_PIT_STEP,           //云台 pitch 轴回中
    YAW_BACK_STEP,           //云台 yaw 轴回中
    BACK_IS_OK,              //云台回中完毕
} gimbal_back_e;

/**
  * @brief     云台控制数据结构体
  */
typedef struct
{
    gimbal_mode_e ctrl_mode; //云台当前控制模式
    gimbal_mode_e last_mode; //云台上次控制模式

    action_mode_e ac_mode;   //云台控制信号输入模式

    uint8_t  no_action_flag; //无控制信号标志
    uint32_t no_action_time; //无控制信号时间

    float ecd_offset_angle;  //云台初始编码器值
    float yaw_offset_angle;  //云台初始 yaw 轴角度
    float pit_offset_angle;  //云台初始 yaw 轴角度
} gimbal_yaw_t;

//自瞄发送结构体
typedef __packed struct
{
    uint16_t head;						//帧头
    float pitchAngleGet;    	//pitch轴角度
    float yawAngleGet;      	//yaw轴角度
    uint8_t rotateDricetion;  //旋转方向
    float timeBais;         	//预测时间偏置
    float compensateBais;   	//弹道补偿偏置
    uint8_t gimbal_mode;	 		//云台模式
}send_frame;


//自瞄接收结构体
typedef __packed struct
{
    uint16_t head;  				 //帧头
    float pitchAngleSet;    //pitch轴角度设定值
    float yawAngleSet;      //yaw轴角度设定值
    float targetAngle;      //目标装甲板角度
    uint8_t shootCommand;   //发射指令
}recv_frame;

extern gimbal_yaw_t gim;
extern imu_t        imu;

/* 云台 PID 控制相关数据 */
extern float yaw_angle_ref;
extern float pit_angle_ref;
extern float yaw_angle_fdb;
extern float pit_angle_fdb;
extern float yaw_speed_ref;
extern float pit_speed_ref;
/* 云台相对角度 */
extern float yaw_relative_angle;
extern float pit_relative_angle;
extern char color_flag;


/**
  * @brief     读取云台中点的编码器偏移数据
  * @param     pit_offset: pitch 轴编码器数据指针
  * @param     yaw_offset: yaw 轴编码器数据指针
  */
static void read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset);
/**
  * @brief     获取云台传感器信息
  */
static void get_gimbal_information(void);
/**
  * @brief     获取云台控制模式
  */
static void get_gimbal_mode(void);
/**
  * @brief     云台控制参数初始化
  */
static void gimbal_init_param(void);
/**
  * @brief     云台回中初始化模式处理函数
  */
static void gimbal_init_handle(void);
/**
  * @brief     云台无控制信号输入模式处理函数
  */

static void gimbal_noaction_handle(void);
/**
  * @brief     云台跟随编码器闭环控制处理函数
  */
static void gimbal_loop_handle(void);

/**
  * @brief     云台自定义控制代码接口
  */
void gimbal_custom_control(void);
/**
  * @brief     云台自瞄控制代码接口
  */
void gimbal_auto_control(void);
/**
  * @brief     云台 yaw 轴位置闭环控制
  */
void gimbal_yaw_control(void);
/**
  * @brief     云台 pitch 轴位置闭环控制
  */
void gimbal_pitch_control(void);
/**
  * @brief     发射机构自定义控制代码接口
  */
void shoot_custom_control(void);
/**
  * @brief     开关摩擦轮控制函数
  */
void turn_on_off_friction_wheel(void);

void pid_reset_manual(void);

void pid_reset_auto(void);

extern send_frame auto_tx_data;
#endif //BOARD_C_INFANTRY_GIMBAL_H
