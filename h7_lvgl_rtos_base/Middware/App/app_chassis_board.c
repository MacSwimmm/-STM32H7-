#include "app_chassis_board.h"
#include "bsp_uart.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "math.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern uint16_t uart1_delay_count;

/* 底盘运动数据 */
chassis_move_t chassis_move;

/**
  * @brief          底盘测量数据更新
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL) return;
    
    for(uint8_t i = 0; i < 4; i++)
    {
        //更新电机速度
        chassis_move_update->chassis_motor_MG370[i].speed = Encoder_Rpm_Get(i);
    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    //底盘速度环pid值
    const static double chas_speed_pid_param[3] = {MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI, MOTOR_SPEED_PID_KD};
    
    Encoder_Init();
    Motor_Init();
		uart_init(&huart4,  UART_DMA_RX); //陀螺仪串口
    
    //底盘电机pid初始化
    for(uint8_t i = 0; i < 4; i++)
    {
        PID_init(&chassis_move_init->chas_speed_pid_MG370[i], PID_POSITION, chas_speed_pid_param, MOTOR_SPEED_PID_MAX_OUT, MOTOR_SPEED_PID_MAX_IOUT);
    }
  
    
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
    
}

/******************************** 核心 ********************************
  * @brief          控制循环，根据控制设定值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  *********************************************************************/
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    
    for(uint8_t i = 0; i < 4; i++)
    {
        //PID计算，得出值
        PID_Calculate(&chassis_move_control_loop->chas_speed_pid_MG370[i], 
                     chassis_move_control_loop->chassis_motor_MG370[i].speed, 
                     chassis_move_control_loop->chassis_motor_MG370[i].speed_set);
        
        //输出
        Motor_SetPWM(chassis_move_control_loop->chas_speed_pid_MG370[i].Out, i);
    }
    
}



void chassis_task(void *pvParameters)
{
    chassis_init(&chassis_move);  //底盘初始化
    
    while(1)
    {
        chassis_feedback_update(&chassis_move); //底盘数据更新
        //my_uart_printf(&huart1, UART_DMA_TX, "FL:%d, FR:%d, RL:%d, RR:%d\r\n", 
        //               chassis_move.chassis_motor_MG370[0].speed,
        //               chassis_move.chassis_motor_MG370[1].speed,
        //               chassis_move.chassis_motor_MG370[2].speed,
        //               chassis_move.chassis_motor_MG370[3].speed);
            
        chassis_control_loop(&chassis_move);    //底盘控制PID计算
        
//        osDelay(10);
    }
    
}
