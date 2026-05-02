#include "app_chassis_board.h"
#include "app_Navigation.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_GPS.h"
#include "usart.h"

chassis_move_t chassis_move;

float Chassis_Vx_set = 0.0f;
float Chassis_Vy_set = 0.0f;
float Chassis_Wz_set = 0.0f;

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL)
	{
		return;
	}

	QMC5883_GetAngles(&chassis_move_update->qmc_debug_data);

	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_move_update->chassis_motor_MG370[i].speed = Encoder_Rpm_Get(i);
	}
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
	const static double chas_speed_pid_param[3] =
	{
		MOTOR_SPEED_PID_KP,
		MOTOR_SPEED_PID_KI,
		MOTOR_SPEED_PID_KD
	};

	Encoder_Init();
	Motor_Init();
	uart_init(&huart2, UART_DMA_ToIdle_RX);

	for (uint8_t i = 0; i < 4; i++)
	{
		PID_init(&chassis_move_init->chas_speed_pid_MG370[i], PID_POSITION,
				 chas_speed_pid_param, MOTOR_SPEED_PID_MAX_OUT, MOTOR_SPEED_PID_MAX_IOUT);
	}
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	
	Navigation_Update_Loop();//导航更新循环
	chassis_move_control_loop->chassis_motor_MG370[0].speed_set = Chassis_Vx_set + Chassis_Vy_set + Chassis_Wz_set;
	chassis_move_control_loop->chassis_motor_MG370[1].speed_set = Chassis_Vx_set - Chassis_Vy_set - Chassis_Wz_set;
	chassis_move_control_loop->chassis_motor_MG370[2].speed_set = Chassis_Vx_set - Chassis_Vy_set + Chassis_Wz_set;
	chassis_move_control_loop->chassis_motor_MG370[3].speed_set = Chassis_Vx_set + Chassis_Vy_set - Chassis_Wz_set;

	for (uint8_t i = 0; i < 4; i++)
	{
		PID_Calculate(&chassis_move_control_loop->chas_speed_pid_MG370[i],
					  chassis_move_control_loop->chassis_motor_MG370[i].speed,
					  chassis_move_control_loop->chassis_motor_MG370[i].speed_set);

		Motor_SetPWM((int16_t)chassis_move_control_loop->chas_speed_pid_MG370[i].Out, i);
	}
}

void chassis_task(void *pvParameters)
{
	chassis_init(&chassis_move);
	QMC5883_Init();
	GPS_Init();
	//目标点写在这		
	static GPS_Point_t route[3] = {
	     {26.449591,106.650651},   // 航点1：十进制度纬度，经度
	     {26.449698, 106.650615},
			{26.449820, 106.650896}			 
	  };
	 Navigation_Set_Route_Loop(route, 3);
	//Navigation_Set_Route();
	//Navigation_Set_Target(26.449591,106.650651);
	
		
		
	while (1)
	{
		chassis_feedback_update(&chassis_move);
		//Motor_SetAllPWM(20,20,20,20);
		//Chassis_Vx_set = 10;
//		Chassis_Vy_set = 0;
//		Chassis_Wz_set = 0;

		chassis_control_loop(&chassis_move);
		osDelay(10);  	
	}
}
