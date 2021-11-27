//
// Created by HLiamso on 2021-11-14.
//

#ifndef TRANSMISSION_H
#define TRANSMISSION_H
#include "cmsis_os.h"
#include "gpio.h"
#include "Gimbal.h"
#include "keyboard.h"
/**
  * @brief     CAN 设备发送和接收 ID 枚举
  */

/**
  * @brief     解析后的遥控器数据结构体
  */
typedef struct
{
    /* 遥控器的通道数据，数值范围：-660 ~ 660 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧左右
    int16_t ch4;   //左侧上下

    /* 遥控器的拨杆数据，上中下分别为：1、3、2 */
    uint8_t sw1;   //左侧拨杆
    uint8_t sw2;   //右侧拨杆

    /* PC 鼠标数据 */
    struct
    {
        /* 鼠标移动相关 */
        int16_t x;   //鼠标平移
        int16_t y;   //鼠标上下
        /* 鼠标按键相关，1为按下，0为松开 */
        uint8_t l;   //左侧按键
        uint8_t r;   //右侧按键
    }mouse;

    /* PC 键盘按键数据 */
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

    /* 遥控器左侧拨轮数据 */
    int16_t wheel;
}rc_type_t;
/**
  * @brief     遥控器拨杆数据枚举
  */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};


/**
  * @brief     底盘控制模式枚举
  */
typedef enum
{
    CHASSIS_STOP,          //底盘停止
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_OPEN_LOOP,     //底盘开环
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_FIXED_ROUTE,   //底盘固定路线
    CHASSIS_SPIN,          //底盘陀螺模式
} chassis_mode_e;
/**
  * @brief     底盘控制数据结构体
  */

typedef struct
{
    /* 底盘控制模式相关 */
    chassis_mode_e  mode;       //当前底盘控制模式
    chassis_mode_e  last_mode;  //上次底盘控制模式

    /* 底盘移动速度相关数据 */
    float           vx;         //底盘前后速度
    float           vy;         //底盘左右速度
    float           vw;         //底盘旋转速度


    uint8_t         last_sw2;
} chassis_t;




#endif //TRANSMISSION_H
