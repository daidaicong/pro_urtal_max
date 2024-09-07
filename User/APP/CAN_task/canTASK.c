#include "chassis_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern const chassis_move_t* local_chassis_move;

void CANserve_task(void *pvParameters)
{
    vTaskDelay(500);
    //底盘初始化
    while(1)
    {
        vTaskDelay(1);
        CAN_CMD_WHEEL(local_chassis_move->right_leg.wheel_motor.give_current,local_chassis_move->left_leg.wheel_motor.give_current);
        vTaskDelay(1);
        chassis_ctrl_motor(1, 0, 0,0, 0, local_chassis_move->right_leg.front_joint.tor_set);
        chassis_ctrl_motor(2, 0, 0,0, 0, local_chassis_move->right_leg.back_joint.tor_set);
        vTaskDelay(1);
        chassis_ctrl_motor(3, 0, 0,0, 0, local_chassis_move->left_leg.front_joint.tor_set);
        chassis_ctrl_motor(4, 0, 0,0, 0, local_chassis_move->left_leg.back_joint.tor_set);
        // CAN_CMD_CHASSIS(0,0,0,0);
        // vTaskDelay(20);
    }
}   