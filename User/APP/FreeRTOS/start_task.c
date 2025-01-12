/**
  ****************************RM Warrior 2023****************************
  * @file       start_task.c/h
  * @brief      启动任务，将一个个任务开启，分配资源，给定任务优先级,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/1/4         pxx              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "adc.h"

#include "INS_task.h"
#include "user_task.h"
#include "voltage_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "observe_task.h"
extern void CANserve_task(void *pvParameters);

#define OB_TASK_PRIO 10
#define OB_STK_SIZE 512
static TaskHandle_t OBTask_Handler;

#define CAN_TASK_PRIO 15
#define CAN_STK_SIZE 128
static TaskHandle_t CANTask_Handler;

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 128
TaskHandle_t ChassisTask_Handler;

#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 256
TaskHandle_t INSTask_Handler;

#define User_TASK_PRIO 4
#define User_STK_SIZE 512
static TaskHandle_t UserTask_Handler;

#define VOLTAGE_TASK_PRIO 11
#define VOLTAGE_TASK_SIZE 128
static TaskHandle_t VoltageTask_Handler;

#define Detect_TASK_PRIO 10
#define Detect_STK_SIZE 128
static TaskHandle_t DetectTask_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();//进入临界区

    xTaskCreate((TaskFunction_t)INSTask,
                (const char *)"INSTask",
                (uint16_t)INS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);

    xTaskCreate((TaskFunction_t)Observe_task,
                (const char *)"OBTask",
                (uint16_t)OB_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)OB_TASK_PRIO,
                (TaskHandle_t *)&OBTask_Handler);

    xTaskCreate((TaskFunction_t)CANserve_task,
                (const char *)"CANTask",
                (uint16_t)CAN_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CAN_TASK_PRIO,
                (TaskHandle_t *)&CANTask_Handler);

    xTaskCreate((TaskFunction_t)UserTask,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);
	
    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);

    xTaskCreate((TaskFunction_t)battery_voltage_task,
                (const char *)"VoltageTask",
                (uint16_t)VOLTAGE_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)VOLTAGE_TASK_PRIO,
                (TaskHandle_t *)&VoltageTask_Handler);

    xTaskCreate((TaskFunction_t)detect_task,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Detect_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}
