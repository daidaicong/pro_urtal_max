
#include "observe_task.h"

KalmanFilter_t vaEstimateKF;	  

float vaEstimateKF_F[4] = {1.0f, 0.005f, 
                           0.0f, 1.0f};	   

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    

float vaEstimateKF_Q[4] = {5.0f, 0.0f, 
                           0.0f, 5.0f};    

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  300.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	
														 															 

float vel_acc[2]; 
uint32_t OBSERVE_TIME=5;
fp32 v_real;
fp32 aver_v;
fp32 diff_v;
// fp32 x_real;

extern const chassis_move_t* local_chassis_move;
extern const fp32 *local_imu_gyro;	  //imu数据					 
void 	Observe_task(void)
{
  const volatile fp32 *angle;
  const volatile fp32 *gyro;

  angle = get_INS_angle_point();
  gyro = get_gyro_data_point();

  local_chassis_move = get_chassis_control_point();
	// while(local_chassis_move->touchingGroung == false || local_chassis_move->is_conversely == true)
	// {
	//   //st
	// }

	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	//static float aver_v=0.0f;
  static float last_v=0.0f;
	xvEstimateKF_Init(&vaEstimateKF);
	
  TickType_t Chassis_LastWakeTime = xTaskGetTickCount();
  while(1)
	{  
		vTaskDelayUntil(&Chassis_LastWakeTime, 5);
		
		// wr =    -local_chassis_move->right_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
    //         + *(gyro+INS_GYRO_Y_ADDRESS_OFFSET) \
    //         + local_chassis_move->right_leg.angle_dot;
    wr =    -local_chassis_move->right_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
            + local_chassis_move->right_leg.angle_dot;        
    vrb=    wr*WHEEL_R \
            +local_chassis_move->right_leg.leg_length*local_chassis_move->state_ref.theta_dot*arm_cos_f32(local_chassis_move->state_ref.theta)\
            +local_chassis_move->right_leg.length_dot*arm_sin_f32(local_chassis_move->state_ref.theta);//
    //
    // wl =     local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
    //          + *(gyro+INS_GYRO_Y_ADDRESS_OFFSET) \
    //          + local_chassis_move->left_leg.angle_dot;

    wl =     local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
             + local_chassis_move->left_leg.angle_dot;

    vlb=    wl*WHEEL_R \
                +local_chassis_move->left_leg.leg_length*local_chassis_move->state_ref.theta_dot*arm_cos_f32(local_chassis_move->state_ref.theta)\
                +local_chassis_move->left_leg.length_dot*arm_sin_f32(local_chassis_move->state_ref.theta);//
                
    //printf("%f\r\n",(wl+wr)*WHEEL_R /2);
    last_v = aver_v;
    aver_v=(vrb+vlb)/2.0f;//平均速度

    //printf("%f,%f,%f,\r\n",local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed_rpm/2160.0f,-*(gyro+INS_GYRO_Y_ADDRESS_OFFSET),-local_chassis_move->left_leg.angle_dot);
    diff_v = (wr - wl)*WHEEL_R - local_chassis_move->wz_set*2*MOTOR_DISTANCE_TO_CENTER;
    
    if(diff_v >2.0f || diff_v <-2.0f){
      //滑了
      // 调整矩阵R
      vaEstimateKF.R_data[0] = 5000.0f;
    }
    else{
      vaEstimateKF.R_data[0] = 250.0f;
    }
    xvEstimateKF_Update(&vaEstimateKF,*(local_chassis_move->chassis_imu_accel+INS_ACCEL_X_ADDRESS_OFFSET) - 0.4f,aver_v);
    //xvEstimateKF_Update(&vaEstimateKF,(aver_v - last_v)/0.005,aver_v);
    //printf("%f,%f\r\n",(*(local_chassis_move->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET)) - 0.4,aver_v);
    v_real = vel_acc[0];
    //10 ms
    // x_real;
    // x_real += v_real * 0.005f;
    //fp32 wz = 0.5f * (wr - wl)*WHEEL_R / MOTOR_DISTANCE_TO_CENTER;

    //printf("%f,%f\r\n",v_real,x_real);
    // 调整矩阵R
    // rebuild matrix R
    //kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 观测维度
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    
    EstimateKF->MeasuredVector[0] =	vel;
    EstimateKF->MeasuredVector[1] = acc;
    		
    Kalman_Filter_Update(EstimateKF);

    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}
fp32 get_KF_Spd(void)
{   	
    return v_real;
}
fp32 get_raw_Spd(void)
{   	
    return aver_v;
}
fp32 get_diff_Spd(void)
{   	
    return diff_v;
}