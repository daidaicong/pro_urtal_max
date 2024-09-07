#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "adc.h"
#include "buzzer.h"
#include "uart1.h"

#include "INS_Task.h"
#include "chassis_task.h"
#include "remote_control.h"
#include "detect_task.h"

#include "voltage_task.h"
#include "Kalman_Filter.h"
#include "bluetooth.h"
#include "kalman_filter.h"

extern void Observe_task(void);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern fp32 get_KF_Spd(void);
extern fp32 get_raw_Spd(void);
extern fp32 get_diff_Spd(void);
#endif




