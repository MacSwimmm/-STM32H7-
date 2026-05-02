#include "bsp_QMC5883.h"


extern I2C_HandleTypeDef hi2c1; 

CalibParams params;

#define RAD_TO_DEG  (180.0 / 3.14159265358979323846)

void QMC5883_Init(void)
{
    
    uint8_t data;
    
    data = 0x0D;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x09, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x0B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x21, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    
    // 初始化时赋安全值，防止未经 Calibration 校准就直接除以0报错
    params.offset_x = 0; params.offset_y = 0; params.offset_z = 0;
    params.scale_x = 1.0f; params.scale_y = 1.0f; params.scale_z = 1.0f;
		Magnetometer_Calibration();//磁力计校准30s
}

void QMC5883_ReadRawData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    
    
    HAL_I2C_Mem_Read(&hi2c1, QMC5883_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);

    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);
}

void Magnetometer_Calibration(void)
{
    static int16_t min_x = 32767;  // 初始化为一个极大值
    static int16_t max_x = -32768; // 初始化为一个极小值
    static int16_t min_y = 32767;
    static int16_t max_y = -32768;
    static int16_t min_z = 32767;
    static int16_t max_z = -32768;

    int16_t temp_hx, temp_hy, temp_hz;
    
    // 原理：带着车绕原地转最大的几圈，记录全域磁场最大最小阈值
    for (int i = 0; i < 1000; i++)
    {
        QMC5883_ReadRawData(&temp_hx, &temp_hy, &temp_hz);

        if (temp_hx < min_x) min_x = temp_hx;
        if (temp_hx > max_x) max_x = temp_hx;
        if (temp_hy < min_y) min_y = temp_hy;
        if (temp_hy > max_y) max_y = temp_hy;
        if (temp_hz < min_z) min_z = temp_hz;
        if (temp_hz > max_z) max_z = temp_hz;

        //  校准中LED 闪烁 (每 10 次循环，约 300ms 翻转一次)
        if (i % 10 == 0) 
        {
#if defined(LED_GPIO_Port) && defined(LED_Pin)
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#else
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
#endif
        }

        HAL_Delay(30); 
    }

    //  校准完成，LED 常亮
#if defined(LED_GPIO_Port) && defined(LED_Pin)
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); 
#else
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
#endif

    params.offset_x = (max_x + min_x) / 2.0f;
    params.offset_y = (max_y + min_y) / 2.0f;
    params.offset_z = (max_z + min_z) / 2.0f;

    float scale_x = (max_x - min_x) / 2.0f;
    float scale_y = (max_y - min_y) / 2.0f;
    float scale_z = (max_z - min_z) / 2.0f;

    float max_scale = scale_x;
    if (scale_y > max_scale) max_scale = scale_y;
    if (scale_z > max_scale) max_scale = scale_z;

    // 防止除以 0 崩溃
    if(max_scale > 0) {
        params.scale_x = scale_x / max_scale;
        params.scale_y = scale_y / max_scale;
        params.scale_z = scale_z / max_scale;
    }
}

void QMC5883_Get_CalibrationData(float *hx, float *hy, float *hz)
{
    int16_t temp_hx = 0;
    int16_t temp_hy = 0;
    int16_t temp_hz = 0;
    QMC5883_ReadRawData(&temp_hx, &temp_hy, &temp_hz);

    *hx = ((float)temp_hx - params.offset_x) * params.scale_x;
    *hy = ((float)temp_hy - params.offset_y) * params.scale_y;
    *hz = ((float)temp_hz - params.offset_z) * params.scale_z;
}

void QMC5883_GetAngles(EulerAngles *angles)
{
    static float x = 0;
    static float y = 0;
    static float z = 0;

    QMC5883_Get_CalibrationData(&x, &y, &z);

    // 计算偏航角 (Yaw) -- 在平面运动通常只要看这个，这是代替双天线的关键参数
    angles->yaw = atan2((double)y, (double)x) * RAD_TO_DEG;
    if(angles->yaw < 0) angles->yaw += 360.0f;

    // 计算俯仰角 (Pitch)
    angles->pitch = atan2((double)y, sqrt(x*x + z*z)) * RAD_TO_DEG;

    // 计算横滚角 (Roll)
    angles->roll = atan2((double)x, sqrt(y*y + z*z)) * RAD_TO_DEG;

}




