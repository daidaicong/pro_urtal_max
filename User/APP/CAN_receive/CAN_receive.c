#include "CAN_Receive.h"

// #include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "detect_task.h"
#include "stdio.h"
#include "math.h"
//大疆底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
/////////////////////////////DM电机专区//////////////////////////////////
//dm8009电机数据读取
void Damiao_Motor_Enable(uint8_t id){
    CanTxMsg TxMessage;
    TxMessage.StdId = id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFC;
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}
void Damiao_Motor_Disable(uint8_t id){
    CanTxMsg TxMessage;
    TxMessage.StdId = id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFD;
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}
void Damiao_Motor_CleanError(uint8_t id){
    CanTxMsg TxMessage;
    TxMessage.StdId = id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFB;
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}
static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void get_dm8009_motor_measure(dm8009_motor_measure_t* ptr, CanRxMsg* rx_message)
{
    ptr->err = rx_message->Data[0]>>4;
    uint16_t tmp_pos = (int16_t) (((uint16_t)(rx_message->Data[1]) <<8) | ((uint16_t)(rx_message->Data[2])));
    ptr->pos = uint_to_float(tmp_pos,-12.5,12.5,16);

    uint16_t tmp_spd = (rx_message->Data[3]<<4 | (rx_message->Data[4]>>4) );
    ptr->speed = uint_to_float(tmp_spd,-45,45,12);

    uint16_t tmp_tor=((rx_message->Data[4]&0xF)<<8)| rx_message->Data[5];
    ptr->tor = uint_to_float(tmp_tor,-20,20,12);

    ptr->tmos_tmper = rx_message->Data[6];
    ptr->coil_tmper = rx_message->Data[7];
}

//发送
void chassis_ctrl_motor(uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
{
int16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
CanTxMsg txmsg;
pos_tmp =float_to_uint(_pos, -12.5, 12.5, 16);
vel_tmp = float_to_uint(_vel, -45, 45, 12);
kp_tmp =float_to_uint(_KP, 0, 500, 12);
kd_tmp =float_to_uint(_KD, 0, 5, 12);
tor_tmp = float_to_uint(_torq,-20, 20, 12);

txmsg.StdId =id;
txmsg.IDE = CAN_ID_STD;
txmsg.RTR = CAN_RTR_DATA;
txmsg.DLC = 0x08;
txmsg.Data[0]= (pos_tmp >>8);
txmsg.Data[1]= pos_tmp;
txmsg.Data[2]= (vel_tmp >>4);
txmsg.Data[3]= ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
txmsg.Data[4]= kp_tmp;
txmsg.Data[5]= (kd_tmp>> 4);
txmsg.Data[6]= ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
txmsg.Data[7]= tor_tmp;
 
uint8_t mail = CAN_Transmit(CHASSIS_CAN, &txmsg);
//printf("%d\r\n",mail);
}
//DM8009
void CAN_CMD_CHASSIS(int16_t right_front_motor, int16_t right_back_motor, int16_t left_front_motor, int16_t left_back_motor)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;//0x3FE
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = right_front_motor ;
    TxMessage.Data[1] = right_front_motor >> 8;
    TxMessage.Data[2] = right_back_motor ;
    TxMessage.Data[3] = right_back_motor >> 8;
    TxMessage.Data[4] = left_front_motor ;
    TxMessage.Data[5] = left_front_motor >> 8;
    TxMessage.Data[6] = left_back_motor ;
    TxMessage.Data[7] = left_back_motor >> 8;

    uint8_t mail = CAN_Transmit(CHASSIS_CAN, &TxMessage);
    //printf("mail:%d\r\n",mail);
}
/////////////////////////////////DM电机END//////////////////////////

//lk9025电机数据读取
void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, CanRxMsg* rx_message)
{
    ptr->tmper = (int8_t)rx_message->Data[1];
    ptr->tor_current = (int16_t)(rx_message->Data[2] | rx_message->Data[3] << 8);
    ptr->speed = (int16_t)(rx_message->Data[4] | rx_message->Data[5] << 8);
    ptr->pos = (uint16_t)(rx_message->Data[6] | rx_message->Data[7] << 8);
}

//统一处理can接收函数
static void CAN_hook(CanRxMsg *rx_message);
//声明电机变量
//static motor_measure_t motor_right, motor_left, motor_joint[4];
static lk9025_motor_measure_t motor_right, motor_left;
static dm8009_motor_measure_t motor_joint[4];

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        //printf("TI%d\r\n",CAN_MessagePending(CAN1,CAN_FIFO0));
        for(uint8_t i=0;i<CAN_MessagePending(CAN1,CAN_FIFO0);i++){
           CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
           CAN_hook(&rx1_message); 
        }
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN_hook(&rx2_message);
    }
}

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_solve(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif

//发送驱动轮控制命令，其中rev为保留字节
void CAN_CMD_WHEEL(int16_t right, int16_t left)
{
    CanTxMsg WHEEL_TxMessage;
    WHEEL_TxMessage.StdId = CAN_WHEEL_ALL_ID;
    WHEEL_TxMessage.IDE = CAN_ID_STD;
    WHEEL_TxMessage.RTR = CAN_RTR_DATA;
    WHEEL_TxMessage.DLC = 0x08;
    WHEEL_TxMessage.Data[0] = right;
    WHEEL_TxMessage.Data[1] = (right >> 8);
    WHEEL_TxMessage.Data[2] = left;
    WHEEL_TxMessage.Data[3] = (left >> 8);
    WHEEL_TxMessage.Data[4] = 0;
    WHEEL_TxMessage.Data[5] = 0;
    WHEEL_TxMessage.Data[6] = 0;
    WHEEL_TxMessage.Data[7] = 0;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    uint8_t mail =  CAN_Transmit( GIMBAL_CAN,  &WHEEL_TxMessage );
    //printf("%d\r\n",mail);
#endif

}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &WHEEL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void)
{
    return &motor_right;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void)
{
    return &motor_left;
}
//返回关节电机变量地址，通过指针方式获取原始数据
const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i)
{
    return &motor_joint[(i & 0x03)];
}

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook(CanRxMsg *rx_message)
{
    //printf("%d\r\n",rx_message->StdId);
    switch (rx_message->StdId)
    {
    case CAN_RIGHT_MOTOR_ID:
    {
        //处理电机数据宏函数
        //get_motor_measure(&motor_right, rx_message);
        get_lk9025_motor_measure(&motor_right,rx_message);
        //记录时间
        detect_hook(WHEEL_MOTOR5_TOE);
        break;
    }
    case CAN_LEFT_MOTOR_ID:
    {
        //处理电机数据宏函数
        //get_motor_measure(&motor_left, rx_message);
        get_lk9025_motor_measure(&motor_left,rx_message);
        detect_hook(WHEEL_MOTOR6_TOE);
        
        break;
    }
    case CAN_dm8009_M1_ID:
    case CAN_dm8009_M2_ID:
    case CAN_dm8009_M3_ID:
    case CAN_dm8009_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - 1;
        //处理电机数据宏函数
        get_dm8009_motor_measure(&motor_joint[i], rx_message);
        //记录时间
        detect_hook(CHASSIS_MOTOR1_TOE + i);
        //printf("%d\r\n",rx_message->StdId);
        //输出位置修正
        if(rx_message->StdId == CAN_dm8009_M1_ID ){
            motor_joint[0].pos -= 0.689898f;
            motor_joint[0].pos = 3.141593/2 - motor_joint[i].pos;
        }
        else if(rx_message->StdId == CAN_dm8009_M2_ID){
            motor_joint[1].pos += 0.504883f;
            motor_joint[1].pos = 3.141593/2 - motor_joint[i].pos;
        }
        else if(rx_message->StdId == CAN_dm8009_M3_ID){
            motor_joint[2].pos += 1.144236f;
            motor_joint[2].pos += 3.141593/2;
        }
        else if(rx_message->StdId == CAN_dm8009_M4_ID)
        {
            motor_joint[3].pos += 0.473602f;
            motor_joint[3].pos += 3.141593/2;
        }
        break;
    }
    default:
    {
        break;
    }
    }
}
