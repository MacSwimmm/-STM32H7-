#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "main.h"
#include "pid.h"

#define chassis_board_task 1

/* 底盘电机速度PID */	/***************待定***************/
#define MOTOR_SPEED_PID_KP 				0.0f
#define MOTOR_SPEED_PID_KI 				0.0f
#define MOTOR_SPEED_PID_KD				0.0f
#define MOTOR_SPEED_PID_MAX_OUT 	120.0f
#define MOTOR_SPEED_PID_MAX_IOUT	40.0f

typedef struct
{
    double speed;
    double speed_set;
    double angle;
    double angle_set;
    
} chassis_motor_t;

typedef struct
{
    const double *chassis_INS_angle;              //取陀螺仪解算出的欧拉角指针
    PID_t chas_speed_pid_MG370[4];                //底盘电机速度pid ,0为前左，1为前右，2为后左，3为后右
    
    chassis_motor_t chassis_motor_MG370[4];       //底盘电机数据,0为前左，1为前右，2为后左，3为后右
    
} chassis_move_t;

extern void chassis_task(void *pvParameters);

#endif

