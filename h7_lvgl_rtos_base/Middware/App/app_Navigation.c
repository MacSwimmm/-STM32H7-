#include "app_Navigation.h"
#include "bsp_GPS.h"
#include "app_Chassis_board.h"
#include "bsp_QMC5883.h"
#include <math.h>

#define PI 3.14159265358979323846f

Navigation_State_t nav_state = {0};

/* 距离PID参数 (这里只用P控) */
static float Kp_dist = 5.0f;     // 米距 -> 速度 单位转化比例
static float Max_speed = 30.0f;  // 限制最大平移速度 (根据底盘的量程微调)

/* 航向偏角PID参数 */
static float Kp_yaw = 0.5f;      // 角度误差 -> 角速度 比例
static float Max_wz = 15.0f;     // 限制最大旋转角速度
//调整车头对准目标的速度

static void Navigation_Load_Route(GPS_Point_t *waypoints, uint8_t count, uint8_t loop_enable)
{
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    if (count == 0) return;

    for (uint8_t i = 0; i < count; i++) {
        nav_state.route[i] = waypoints[i];
    }

    nav_state.total_waypoints = count;
    nav_state.current_wp_index = 0;
    nav_state.loop_enable = loop_enable;
    nav_state.target_pos = nav_state.route[0];
    nav_state.phase = NAV_PHASE_RUNNING;
    nav_state.is_navigating = 1;
}

// 工具函数：NMEA(ddmm.mmmm) 转换为 十进制经纬度坐标 (DD.DDDD)
static double NMEA_To_Degree(double nmea_cord)
{
    // floor函数用于向下取整，获取整数度数
    double degrees = floor(nmea_cord / 100.0);
    // 取余下的也就是分，换算为十进制的度
    double minutes = nmea_cord - (degrees * 100.0);
    return degrees + (minutes / 60.0);
}

// 工具函数：计算两点经纬度构成的绝对方位角 (0 是正北, 90 是正东)
// 返回 0-360度
static float Calculate_Bearing(double lat1, double lon1, double lat2, double lon2)
{
    double dLon = (lon2 - lon1) * PI / 180.0;
    double lat1Rad = lat1 * PI / 180.0;
    double lat2Rad = lat2 * PI / 180.0;

    double y = sin(dLon) * cos(lat2Rad);
    double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);

    double brng = atan2(y, x);
    brng = brng * 180.0 / PI;
    if(brng < 0) {
        brng += 360.0;
    }
    return (float)brng;
}

// 工具函数：计算两十进制经纬度之间的表面距离(米)
static float Calculate_Distance(double lat1, double lon1, double lat2, double lon2)
{
    // 简平法公式, 适合短距离直线计算 (几十公里内)
    double dLat = (lat2 - lat1) * 111320.0; // 纬度1度大概相当于 111.32km
    double dLon = (lon2 - lon1) * 111320.0 * cos(lat1 * PI / 180.0);
    return (float)sqrt(dLat * dLat + dLon * dLon);
}


// 单目标导航（完全向后兼容原有逻辑，到了就停）
void Navigation_Set_Target(double target_lat, double target_lon)
{
    // 将单目标视为只有1个航点的路线
    nav_state.route[0].lat = target_lat;
    nav_state.route[0].lon = target_lon;
    nav_state.total_waypoints = 1;
    nav_state.current_wp_index = 0;
    nav_state.loop_enable = 0;
    
    // 设置当前目标（与原逻辑一致）
    nav_state.target_pos.lat = target_lat;
    nav_state.target_pos.lon = target_lon;
    nav_state.phase = NAV_PHASE_RUNNING;
    nav_state.is_navigating = 1;
}

// 多目标巡航：按顺序传入航点数组，小车会依次前往每个航点
// 每到达一个中间航点就停留 WAYPOINT_DWELL_MS 毫秒，然后自动出发去下一个
void Navigation_Set_Route(GPS_Point_t *waypoints, uint8_t count)
{
    Navigation_Load_Route(waypoints, count, 0);
}

void Navigation_Set_Route_Loop(GPS_Point_t *waypoints, uint8_t count)
{
    Navigation_Load_Route(waypoints, count, 1);
}

void Navigation_Stop(void)
{
    nav_state.is_navigating = 0;
    nav_state.loop_enable = 0;
    nav_state.phase = NAV_PHASE_IDLE;
    Chassis_Vx_set = 0;
    Chassis_Vy_set = 0;
    Chassis_Wz_set = 0;
}

// ==================================
// 核心更新循环：主函数负责周期调用它
// ==================================
void Navigation_Update_Loop(void)
{
    if (!nav_state.is_navigating) {
        return;
    }

    // ========== 停留等待阶段 ==========
    // 到达中间航点后，在此原地停留，等够时间再出发去下一个
    if (nav_state.phase == NAV_PHASE_DWELLING) {
        // 停留期间保持刹车
        Chassis_Vx_set = 0;
        Chassis_Vy_set = 0;
        Chassis_Wz_set = 0;
        
        // 检查停留时间是否已满
        if (HAL_GetTick() - nav_state.dwell_start_tick >= WAYPOINT_DWELL_MS) {
            // 时间到了：普通巡航前往下一个点，循环巡航在末点回到第一个点
            if (nav_state.current_wp_index + 1 >= nav_state.total_waypoints) {
                nav_state.current_wp_index = 0;
            } else {
                nav_state.current_wp_index++;
            }
            nav_state.target_pos = nav_state.route[nav_state.current_wp_index];
            nav_state.phase = NAV_PHASE_RUNNING;
        }
        return; // 停留期间不执行下方的行驶逻辑
    }

    // ========== 行驶阶段 ==========

    // --- 【提数接口】：向 GPS 模块索要最新解析的内存数据 ---
    PT_GNGGA pGGA = GetGNGGA();
    PT_GPHPR pHPR = GetGPHPR();

    // qf(定位质量): 4是RTK紧耦合，5是浮动解，1是普通定位。纬度<1表示无效空数据
    if (pGGA->qf < 1 || pGGA->lat < 1.0f) {
        // 定位无效时，强制停车以保障安全
        Chassis_Vx_set = 0;
        Chassis_Vy_set = 0;
        Chassis_Wz_set = 0;
        return; 
    }
    nav_state.rtk_quality = pGGA->qf;
    // (在 pGGA->qf 判断通过之后，存一份到 nav_state 供读取)
    // 提取并转化当前GPS位置 -> 十进制实数度数
    nav_state.current_pos.lat = NMEA_To_Degree(pGGA->lat);
    nav_state.current_pos.lon = NMEA_To_Degree(pGGA->lon);
    
    // 单天线，GPS得不到正确的静态车头解算
    // 使用放置在高处的 QMC5883 磁力计覆盖偏航角 (0-360)
    EulerAngles mag_angles;
    QMC5883_GetAngles(&mag_angles);
    nav_state.current_heading = mag_angles.yaw; 

    // 目标 GPS 现已规定直接为十进制实数度数，无需再做 NMEA 转换
    double target_decimal_lat = nav_state.target_pos.lat;
    double target_decimal_lon = nav_state.target_pos.lon;

    // 1. 直线拉扯：计算与目标点的绝对方位角和当前直线距离
    nav_state.target_bearing = Calculate_Bearing(nav_state.current_pos.lat, nav_state.current_pos.lon, target_decimal_lat, target_decimal_lon);
    nav_state.distance_error = Calculate_Distance(nav_state.current_pos.lat, nav_state.current_pos.lon, target_decimal_lat, target_decimal_lon);

    // 2. 距离到达判断   0.2f代表20cm,2f代表2m(可用于无4G ntrip条件下的测试)
    if (nav_state.distance_error < 2.0f) { 
        // 到达当前航点,先刹车
        Chassis_Vx_set = 0;
        Chassis_Vy_set = 0;
        Chassis_Wz_set = 0;
        
        // 判断是否是最后一个航点
        if (nav_state.current_wp_index + 1 >= nav_state.total_waypoints) {
            if (nav_state.loop_enable && nav_state.total_waypoints > 1) {
                // 循环巡航：在最后一个航点也停留，然后切回第一个航点
                nav_state.dwell_start_tick = HAL_GetTick();
                nav_state.phase = NAV_PHASE_DWELLING;
            } else {
                // 默认行为保持不变：到最后一个航点后停车
                Navigation_Stop();
            }
        } else {
            // 还有下一个航点,记录到达时间,进入停留等待阶段
            nav_state.dwell_start_tick = HAL_GetTick();
            nav_state.phase = NAV_PHASE_DWELLING;
        }
        return;
    }

    // 3. 计算相对夹角偏差 (Target 指向的真实方位 - 车头方位) 
    // 并将其归一化到 [-180, +180] 度内
    float angle_diff = nav_state.target_bearing - nav_state.current_heading;
    while (angle_diff > 180.0f)  angle_diff -= 360.0f;
    while (angle_diff <= -180.0f) angle_diff += 360.0f;
    nav_state.heading_error = angle_diff;

    // 4. 【全向平移动力学拆解 (Omni- Crab Walk)】
    // 并非僵硬地等车头转了之后再走Vx。 angle_diff 代表小车还要面对自身偏移的角度。
    // 直接把平向冲击力转化至 Vx(纵向轮滚) 与 Vy(横向侧滑)。
    
    // a. 距离产生动力 (Distance P-Controller)
    float target_v = nav_state.distance_error * Kp_dist;
    if (target_v > Max_speed) target_v = Max_speed;

    // 分解力到左右和前后 
    // (注意：角偏差在数学系里用正余弦换出XY向量)
    float rad_err = angle_diff * PI / 180.0f;
    float out_vx = target_v * cosf(rad_err); 
    float out_vy = target_v * sinf(rad_err);

    // b. 加入车体的柔和转向自适应对准 (Heading P-Controller) 
    float out_wz = angle_diff * Kp_yaw;
    if (out_wz > Max_wz) out_wz = Max_wz;
    if (out_wz < -Max_wz) out_wz = -Max_wz;

    // 实时赋值给底盘
    Chassis_Vx_set = out_vx;
    Chassis_Vy_set = out_vy;
    Chassis_Wz_set = out_wz;
}
