#ifndef __QMC5883_H
#define __QMC5883_H

#include "stm32h7xx_hal.h"
#include <math.h>

// I2C 7位地址为 0x0D，在HAL库中要求左移一位(即8位地址)，因此是 0x1A
#define QMC5883_ADDR 0x1A  

typedef struct {
    float yaw;      // 偏航角 (0~360度)
    float pitch;    // 俯仰角 (-180~180度)
    float roll;     // 横滚角 (-180~180度)
} EulerAngles;

typedef struct {
    float offset_x, offset_y, offset_z;
    float scale_x, scale_y, scale_z;
} CalibParams;

void QMC5883_Init(void);
void QMC5883_ReadRawData(int16_t *x, int16_t *y, int16_t *z);
void QMC5883_GetAngles(EulerAngles *angles);
void Magnetometer_Calibration(void);
void QMC5883_Get_CalibrationData(float *hx, float *hy, float *hz);

#endif
