/*********************************************************************************************************************
* CYT4BB Navigation & Decision System
* Copyright (c) 2024
*
* 文件名称          nav_decision.h
* 功能描述          导航与决策系统头文件
*                   - 路点存储与管理
*                   - 航向偏差计算
*                   - 摄像头/GPS导航模式切换
* 版本信息          V1.0
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
*
* 修改记录
* 日期              作者                备注
* 2024-01-25       AI                  first version
********************************************************************************************************************/

#ifndef _NAV_DECISION_H_
#define _NAV_DECISION_H_

#include "zf_common_typedef.h"

//==================================================== 宏定义 ====================================================

// 路点配置
#define NAV_MAX_WAYPOINTS           (100)           // 最大路点数量
#define NAV_WAYPOINT_REACH_DIST     (1.0f)          // 到达路点判定距离 (米)
#define NAV_LOOKAHEAD_DIST          (2.0f)          // 前瞻距离 (米)

// 导航模式
#define NAV_MODE_CAMERA             (0)             // 摄像头导航模式
#define NAV_MODE_GPS                (1)             // GPS导航模式
#define NAV_MODE_MIXED              (2)             // 混合导航模式

// 混合导航策略参数
#define NAV_CAMERA_MIN_VALID_ROWS   (20)            // 摄像头有效行最小阈值
#define NAV_CAMERA_MAX_AGE_MS       (300)           // 摄像头数据最大允许年龄（毫秒）
#define NAV_GPS_HDOP_ACCEPT         (3.5f)          // GPS接管时允许的最大HDOP
#define NAV_STEER_SLEW_LIMIT        (6.0f)          // 每次更新允许的最大转向变化量(度)
#define NAV_SOURCE_MIN_HOLD_CYCLES (5)             // 切换后最少保持的更新周期数
#define NAV_GPS_HEADING_MIN_SPEED   (0.5f)          // 低于该速度时优先不用GPS航向
#define NAV_GPS_MAX_AGE_MS          (1500)          // GPS数据最大允许年龄（毫秒）
#define NAV_DEFAULT_CAMERA_SPEED    (0.8f)          // 无路点时视觉导航默认目标速度
#define NAV_CAMERA_OFFSET_JUMP_MAX  (45)            // 摄像头偏移突变阈值（像素）
#define NAV_CAMERA_SLOPE_JUMP_MAX   (500)           // 摄像头斜率突变阈值（放大100倍）
#define NAV_CAMERA_ROAD_LOST_TYPE   (3)             // 视觉模块 ROAD_LOST 对应值

// 摄像头丢线判定
#define NAV_CAMERA_LOST_THRESHOLD   (5)             // 连续丢线帧数阈值
#define NAV_CAMERA_RECOVER_THRESHOLD (3)            // 连续恢复帧数阈值

// 航向控制参数
#define NAV_HEADING_KP              (1.0f)          // 航向偏差比例系数
#define NAV_HEADING_KD              (0.1f)          // 航向偏差微分系数
#define NAV_MAX_STEER_ANGLE         (30.0f)         // 最大转向角度 (度)

// 数学常量
#define NAV_PI                      (3.14159265f)
#define NAV_DEG_TO_RAD              (0.01745329f)   // PI/180
#define NAV_RAD_TO_DEG              (57.2957795f)   // 180/PI

//==================================================== 枚举定义 ====================================================

// 导航状态枚举
typedef enum
{
    NAV_STATE_IDLE = 0,             // 空闲状态
    NAV_STATE_RUNNING,              // 正在导航
    NAV_STATE_PAUSED,               // 暂停
    NAV_STATE_REACHED,              // 到达终点
    NAV_STATE_ERROR                 // 错误状态
} nav_state_enum;

// 路点类型枚举
typedef enum
{
    WAYPOINT_NORMAL = 0,            // 普通路点
    WAYPOINT_STOP,                  // 停车点
    WAYPOINT_SLOW,                  // 减速点
    WAYPOINT_TURN_L,                // 左转点
    WAYPOINT_TURN_R,                // 右转点
    WAYPOINT_START,                 // 起点
    WAYPOINT_FINISH                 // 终点
} waypoint_type_enum;

// 导航数据源枚举
typedef enum
{
    NAV_SOURCE_CAMERA = 0,          // 摄像头数据
    NAV_SOURCE_GPS,                 // GPS数据
    NAV_SOURCE_FUSED                // 融合数据
} nav_source_enum;

// 任务状态枚举
typedef enum
{
    NAV_TASK_IDLE = 0,              // 空闲/未进入任务
    NAV_TASK_STRAIGHT_RUN,          // 直行阶段
    NAV_TASK_CONE_SLALOM,           // 绕桩/穿桩阶段
    NAV_TASK_MINE_SEARCH,           // 雷区搜索与接近
    NAV_TASK_MINE_ROTATE,           // 雷区内定点旋转
    NAV_TASK_BRIDGE_PASS,           // 单边桥/窄路通过
    NAV_TASK_STAIRS_PASS,           // 台阶/坡道通过
    NAV_TASK_RETURN_HOME            // 返回阶段
} nav_task_enum;

// 导航切换原因枚举
typedef enum
{
    NAV_SWITCH_NONE = 0,            // 无切换/保持当前状态
    NAV_SWITCH_CAMERA_TRACKING,     // 正在使用摄像头跟踪
    NAV_SWITCH_GPS_TRACKING,        // 正在使用GPS跟踪
    NAV_SWITCH_CAMERA_LOST,         // 摄像头丢线切换到GPS
    NAV_SWITCH_CAMERA_RECOVERED,    // 摄像头恢复切回视觉
    NAV_SWITCH_GPS_UNRELIABLE,      // GPS不可靠，回退到视觉
    NAV_SWITCH_GPS_STALE,           // GPS数据过旧，回退到视觉
    NAV_SWITCH_NO_WAYPOINTS,        // 没有可用路点，无法进行GPS导航
    NAV_SWITCH_BOTH_UNAVAILABLE,    // 两种导航源都不可用
    NAV_SWITCH_FORCED_MODE          // 被模式强制选中
} nav_switch_reason_enum;

//==================================================== 结构体定义 ====================================================

// 路点结构体
typedef struct
{
    float x;                        // X坐标 (米)
    float y;                        // Y坐标 (米)
    waypoint_type_enum type;        // 路点类型
    float target_speed;             // 目标速度 (m/s)
} waypoint_struct;

// 路径结构体
typedef struct
{
    waypoint_struct points[NAV_MAX_WAYPOINTS];  // 路点数组
    uint16 count;                   // 路点总数
    uint16 current_index;           // 当前目标路点索引
    uint8 is_loop;                  // 是否循环路径
} path_struct;

// 导航输入数据结构体
typedef struct
{
    // 当前位置 (来自GPS)
    float pos_x;                    // 当前X坐标 (米)
    float pos_y;                    // 当前Y坐标 (米)
    float gps_heading;              // GPS航向角 (度, 北为0)
    uint8 gps_valid;                // GPS数据有效标志
    uint8 gps_reliable;             // GPS数据可靠标志
    uint8 gps_origin_ready;         // GPS原点已设置标志
    float gps_hdop;                 // GPS当前HDOP
    uint32 gps_update_age_ms;       // GPS数据距当前循环的时间差（毫秒）

    // 赛道任务视觉语义（后续摄像头方案扩展的主要输出）
    uint8 blue_zone_detected;       // 是否检测到蓝色任务区域（雷区/台阶）
    uint8 blue_zone_hold_frames;     // 蓝色任务区域保持计数
    uint8 cone_gap_detected;         // 是否检测到可穿越桩间隙
    uint8 cone_gap_hold_frames;      // 桩间隙保持计数
    int16 cone_gap_offset;           // 桩间隙中心相对图像中心偏移
    uint8 mine_box_detected;         // 是否检测到白色雷区边框
    uint8 mine_box_hold_frames;      // 雷区边框保持计数
    int16 mine_box_offset;           // 雷区中心偏移
    uint8 bridge_detected;           // 是否检测到交错单边桥结构
    uint8 bridge_hold_frames;        // 单边桥保持计数
    int16 bridge_center_offset;      // 单边桥可通行中心偏移
    uint16 bridge_distance_est;      // 单边桥相对距离估计（值越小表示越近）
    uint8 bridge_distance_level;     // 单边桥接近等级（0远-3近）
    uint8 bridge_alignment_ok;       // 单边桥是否已基本对准
    uint8 stairs_detected;           // 是否检测到台阶/坡道结构
    uint8 stairs_hold_frames;        // 台阶保持计数
    int16 stairs_center_offset;      // 台阶中心偏移
    uint16 stairs_distance_est;      // 台阶相对距离估计（值越小表示越近）
    uint8 stairs_distance_level;     // 台阶接近等级（0远-3近）
    uint8 stairs_alignment_ok;       // 台阶是否已基本对准

    // 摄像头数据
    int16 camera_center_offset;     // 摄像头中线偏移 (像素, 正=右偏)
    float camera_curvature;         // 赛道曲率估计
    uint8 camera_valid;             // 摄像头数据有效标志 (0=丢线)
    uint8 camera_valid_rows;        // 摄像头有效行数
    uint32 camera_update_age_ms;    // 摄像头数据距当前循环的时间差（毫秒）
    int16 camera_center_slope;      // 摄像头中线斜率（放大100倍）
    uint8 camera_road_type;         // 当前路型（与视觉模块 road_type 对齐）

    // 车辆状态
    float current_speed;            // 当前车速 (m/s)
    float imu_yaw;                  // IMU航向角 (度)
} nav_input_struct;

// 导航输出数据结构体
typedef struct
{
    float steer_angle;              // 目标转向角 (度, 正=右转)
    float target_speed;             // 目标速度 (m/s)

    nav_source_enum active_source;  // 当前激活的数据源
    nav_state_enum state;           // 导航状态
    nav_task_enum current_task;     // 当前任务状态
    nav_switch_reason_enum switch_reason; // 当前切换/保持原因
    uint8 camera_usable;            // 摄像头是否可接管
    uint8 gps_usable;               // GPS是否可接管
    uint8 camera_lost_frames;       // 当前连续视觉丢失计数
    uint8 camera_recover_frames;    // 当前连续视觉恢复计数
    uint8 source_hold_frames;       // 当前数据源已保持的周期数
    uint8 task_hold_frames;         // 当前任务保持计数
    uint32 camera_age_ms;           // 当前视觉数据年龄（毫秒）
    uint8 bridge_execute;           // 是否允许底层进入桥姿态执行
    uint8 jump_execute;             // 是否允许底层执行起跳动作
    uint8 mine_rotate_execute;      // 是否允许底层执行雷区旋转动作

    // 调试信息
    float heading_error;            // 航向偏差 (度)
    float cross_track_error;        // 横向偏差 (米)
    float distance_to_target;       // 到目标点距离 (米)
    float target_heading;           // 目标航向 (度)
    float used_heading;             // 当前导航实际使用的航向 (度)
    float raw_camera_steer;         // 视觉原始转向输出
    float raw_gps_steer;            // GPS原始转向输出
} nav_output_struct;

// 导航控制器结构体
typedef struct
{
    uint8 mode;                     // 导航模式 (CAMERA/GPS/MIXED)
    nav_state_enum state;           // 导航状态
    nav_task_enum current_task;     // 当前任务状态

    // 摄像头丢线计数
    uint8 camera_lost_count;        // 连续丢线计数
    uint8 camera_recover_count;     // 连续恢复计数
    uint8 is_camera_lost;           // 摄像头丢线标志

    // 当前源状态
    nav_source_enum preferred_source;   // 当前偏好的导航源
    nav_source_enum pending_source;     // 预留：待切换的数据源
    uint8 source_hold_count;            // 保持当前源的计数
    uint8 task_hold_count;              // 任务保持计数
    float last_steer_cmd;               // 上一次输出的转向角
    int16 last_camera_offset;           // 上一帧摄像头偏移
    int16 last_camera_slope;            // 上一帧摄像头斜率
    uint8 last_camera_sample_valid;     // 上一帧视觉样本是否有效

    // 航向PD控制
    float last_heading_error;       // 上一次航向偏差

    // 统计信息
    uint32 gps_nav_time_ms;         // GPS导航累计时间
    uint32 camera_nav_time_ms;      // 摄像头导航累计时间
} nav_controller_struct;

//==================================================== 全局变量声明 ====================================================

extern path_struct          nav_path;           // 导航路径
extern nav_controller_struct nav_ctrl;          // 导航控制器
extern nav_output_struct    nav_output;         // 导航输出

//==================================================== 函数声明 ====================================================

// -------------------- 初始化函数 --------------------
void nav_decision_init(void);                   // 导航系统初始化

// -------------------- 路点管理函数 --------------------
uint8 nav_add_waypoint(float x, float y, waypoint_type_enum type, float speed);     // 添加路点
uint8 nav_add_waypoint_current(waypoint_type_enum type, float speed);               // 添加当前位置为路点
void nav_clear_waypoints(void);                 // 清空所有路点
uint16 nav_get_waypoint_count(void);            // 获取路点数量
waypoint_struct* nav_get_waypoint(uint16 index);    // 获取指定路点
void nav_set_loop(uint8 enable);                // 设置循环模式

// -------------------- 导航控制函数 --------------------
void nav_start(void);                           // 开始导航
void nav_stop(void);                            // 停止导航
void nav_pause(void);                           // 暂停导航
void nav_resume(void);                          // 恢复导航
void nav_set_mode(uint8 mode);                  // 设置导航模式

// -------------------- 核心更新函数 --------------------
void nav_update(nav_input_struct *input);       // 导航主更新函数 (每帧调用)

// -------------------- 航向计算函数 --------------------
float nav_calc_target_heading(float cur_x, float cur_y, float tar_x, float tar_y);  // 计算目标航向
float nav_calc_heading_error(float current_heading, float target_heading);          // 计算航向偏差
float nav_calc_distance(float x1, float y1, float x2, float y2);                    // 计算两点距离
float nav_calc_cross_track_error(float cur_x, float cur_y, uint16 wp_index);        // 计算横向偏差

// -------------------- Pure Pursuit 算法 --------------------
float nav_pure_pursuit(float cur_x, float cur_y, float cur_heading, float lookahead);   // Pure Pursuit算法

// -------------------- 模式切换函数 --------------------
void nav_check_camera_status(uint8 camera_valid);   // 检查摄像头状态，自动切换模式
nav_source_enum nav_get_active_source(void);        // 获取当前激活的数据源

// -------------------- 辅助函数 --------------------
float nav_normalize_angle(float angle);         // 角度归一化到 -180~180
float nav_clamp(float value, float min, float max);     // 数值限幅

#endif
