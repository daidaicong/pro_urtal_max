#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"
#include "stm32f4xx.h"
#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x3FE,
    CAN_dm8009_M1_ID = 0x01,
    CAN_dm8009_M2_ID = 0x02,
    CAN_dm8009_M3_ID = 0x03,
    CAN_dm8009_M4_ID = 0x04,

    CAN_RIGHT_MOTOR_ID = 0x141,
    CAN_LEFT_MOTOR_ID = 0x142,
    CAN_WHEEL_ALL_ID = 0x280,
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;           //转子机械角度
    int16_t speed_rpm;      //转子转速
    int16_t given_current;  //实际转矩电流
    uint8_t temperate;      //电机温度
    int16_t last_ecd;       //上次转子机械角度
} motor_measure_t;

//dm8009电机统一数据结构体
typedef struct
{
    uint8_t err;           //err
    float pos;           //位置
    float speed;         //速度
    float tor;           //转矩
    int8_t tmos_tmper;     //tmos温度
    int8_t coil_tmper;     //线圈温度
} dm8009_motor_measure_t;

//lk9025电机统一数据结构体
typedef struct
{
    int16_t speed;                //速度
    int16_t tor_current;          //转矩电流
    uint16_t pos;                 //编码器位置
    int8_t  tmper;                //电机温度
} lk9025_motor_measure_t;

//extern void CAN_CMD_CHASSIS_RESET_ID(void);
extern void Damiao_Motor_Enable(uint8_t id);
extern void Damiao_Motor_Disable(uint8_t id);
extern void Damiao_Motor_CleanError(uint8_t id);
extern void CAN_CMD_CHASSIS(int16_t right_front_motor, int16_t right_back_motor, int16_t left_front_motor, int16_t left_back_motor);
//接收处理函数
extern void get_dm8009_motor_measure(dm8009_motor_measure_t* ptr, CanRxMsg* rx_message);
void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, CanRxMsg* rx_message);

//发送轮子控制命令，其中rev为保留字节
extern void CAN_CMD_WHEEL(int16_t right, int16_t left);
//发送底盘电机控制命令
extern void chassis_ctrl_motor(uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);

//返回右驱动轮电机变量地址，通过指针方式获取原始数据
extern const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void);
//返回左驱动轮电机变量地址，通过指针方式获取原始数据
extern const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void);
//返回关节电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_solve(void);
#endif

#endif
