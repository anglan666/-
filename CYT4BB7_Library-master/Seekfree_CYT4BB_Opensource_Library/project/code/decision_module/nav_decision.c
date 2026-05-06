/*********************************************************************************************************************
* CYT4BB Navigation & Decision System
* Copyright (c) 2024
*
* 文件名称          nav_decision.c
* 功能描述          导航与决策系统实现
*                   - 路点存储与管理
*                   - Pure Pursuit 路径跟踪算法
*                   - 摄像头/GPS导航模式自动切换
* 版本信息          V1.0
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
*
* 修改记录
* 日期              作者                备注
* 2024-01-25       AI                  first version
********************************************************************************************************************/

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "zf_common_headfile.h"
#include "nav_decision.h"
#include "../gps_module/gps_service.h"

//==================================================== 全局变量 ====================================================

path_struct             nav_path;               // 导航路径
nav_controller_struct   nav_ctrl;               // 导航控制器
nav_output_struct       nav_output;             // 导航输出

//==================================================== 内部函数声明 ====================================================

static float nav_camera_to_steer(int16 center_offset, float curvature);
static float nav_gps_to_steer(nav_input_struct *input);
static void nav_update_waypoint_tracking(float cur_x, float cur_y, float heading);
static uint8 nav_camera_is_usable(const nav_input_struct *input);
static uint8 nav_gps_is_usable(const nav_input_struct *input);
static nav_switch_reason_enum nav_get_gps_unusable_reason(const nav_input_struct *input);
static nav_task_enum nav_select_task(const nav_input_struct *input, uint8 camera_usable, uint8 gps_usable);
static uint8 nav_get_task_hold_limit(nav_task_enum task);
static uint8 nav_should_switch_task_immediately(nav_task_enum current_task, nav_task_enum selected_task);
static nav_source_enum nav_select_source(const nav_input_struct *input, uint8 camera_usable, uint8 gps_usable);
static float nav_apply_steer_filter(float steer);
static nav_switch_reason_enum nav_get_hold_reason(const nav_input_struct *input, nav_source_enum source, uint8 camera_usable, uint8 gps_usable);
static float nav_resolve_heading(const nav_input_struct *input);

//==================================================== 初始化函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
// 参数说明     void
// 返回参数     void
// 使用示例     nav_decision_init();
// 备注信息     在系统启动时调用一次
//-------------------------------------------------------------------------------------------------------------------
void nav_decision_init(void)
{
    // 清空路径
    memset(&nav_path, 0, sizeof(nav_path));
    nav_path.count = 0;
    nav_path.current_index = 0;
    nav_path.is_loop = 0;
    
    // 初始化控制器
    memset(&nav_ctrl, 0, sizeof(nav_ctrl));
    nav_ctrl.mode = NAV_MODE_MIXED;             // 默认混合模式
    nav_ctrl.state = NAV_STATE_IDLE;
    nav_ctrl.current_task = NAV_TASK_IDLE;
    nav_ctrl.camera_lost_count = 0;
    nav_ctrl.camera_recover_count = 0;
    nav_ctrl.is_camera_lost = 0;
    nav_ctrl.preferred_source = NAV_SOURCE_CAMERA;
    nav_ctrl.pending_source = NAV_SOURCE_CAMERA;
    nav_ctrl.source_hold_count = 0;
    nav_ctrl.task_hold_count = 0;
    nav_ctrl.last_steer_cmd = 0;
    nav_ctrl.last_camera_offset = 0;
    nav_ctrl.last_camera_slope = 0;
    nav_ctrl.last_camera_sample_valid = 0;
    nav_ctrl.last_heading_error = 0;
    
    // 初始化输出
    memset(&nav_output, 0, sizeof(nav_output));
    nav_output.state = NAV_STATE_IDLE;
    nav_output.current_task = NAV_TASK_IDLE;
    nav_output.active_source = NAV_SOURCE_CAMERA;
    nav_output.switch_reason = NAV_SWITCH_NONE;
}

//==================================================== 路点管理函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     添加路点
// 参数说明     x           X坐标 (米)
// 参数说明     y           Y坐标 (米)
// 参数说明     type        路点类型
// 参数说明     speed       目标速度 (m/s)
// 返回参数     uint8       0-成功 1-失败(路点已满)
// 使用示例     nav_add_waypoint(10.0f, 20.0f, WAYPOINT_NORMAL, 2.0f);
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_add_waypoint(float x, float y, waypoint_type_enum type, float speed)
{
    if(nav_path.count >= NAV_MAX_WAYPOINTS)
    {
        return 1;                               // 路点已满
    }
    
    nav_path.points[nav_path.count].x = x;
    nav_path.points[nav_path.count].y = y;
    nav_path.points[nav_path.count].type = type;
    nav_path.points[nav_path.count].target_speed = speed;
    nav_path.count++;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     添加当前GPS位置为路点
// 参数说明     type        路点类型
// 参数说明     speed       目标速度 (m/s)
// 返回参数     uint8       0-成功 1-失败
// 使用示例     nav_add_waypoint_current(WAYPOINT_NORMAL, 2.0f);
// 备注信息     需要GPS已定位且原点已设置
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_add_waypoint_current(waypoint_type_enum type, float speed)
{
    if(!gps_is_fixed() || !gps_data.origin_set)
    {
        return 1;                               // GPS未定位或原点未设置
    }
    
    return nav_add_waypoint(gps_data.pos_x, gps_data.pos_y, type, speed);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空所有路点
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void nav_clear_waypoints(void)
{
    nav_path.count = 0;
    nav_path.current_index = 0;
    nav_ctrl.state = NAV_STATE_IDLE;
    nav_ctrl.current_task = NAV_TASK_IDLE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取路点数量
// 返回参数     uint16      路点数量
//-------------------------------------------------------------------------------------------------------------------
uint16 nav_get_waypoint_count(void)
{
    return nav_path.count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取指定路点
// 参数说明     index       路点索引
// 返回参数     waypoint_struct*    路点指针，索引无效返回NULL
//-------------------------------------------------------------------------------------------------------------------
waypoint_struct* nav_get_waypoint(uint16 index)
{
    if(index >= nav_path.count)
    {
        return (waypoint_struct*)0;
    }
    return &nav_path.points[index];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置循环模式
// 参数说明     enable      0-不循环 1-循环
//-------------------------------------------------------------------------------------------------------------------
void nav_set_loop(uint8 enable)
{
    nav_path.is_loop = enable;
}

//==================================================== 导航控制函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始导航
//-------------------------------------------------------------------------------------------------------------------
void nav_start(void)
{
    if(nav_path.count > 0 || nav_ctrl.mode != NAV_MODE_GPS)
    {
        nav_path.current_index = 0;
        nav_ctrl.state = NAV_STATE_RUNNING;
        nav_ctrl.current_task = NAV_TASK_STRAIGHT_RUN;
        nav_ctrl.camera_lost_count = 0;
        nav_ctrl.camera_recover_count = 0;
        nav_ctrl.is_camera_lost = 0;
        nav_ctrl.preferred_source = NAV_SOURCE_CAMERA;
        nav_ctrl.pending_source = NAV_SOURCE_CAMERA;
        nav_ctrl.source_hold_count = 0;
        nav_ctrl.last_steer_cmd = 0;
        nav_ctrl.last_camera_offset = 0;
        nav_ctrl.last_camera_slope = 0;
        nav_ctrl.last_camera_sample_valid = 0;
        nav_output.switch_reason = NAV_SWITCH_NONE;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止导航
//-------------------------------------------------------------------------------------------------------------------
void nav_stop(void)
{
    nav_ctrl.state = NAV_STATE_IDLE;
    nav_ctrl.current_task = NAV_TASK_IDLE;
    nav_output.steer_angle = 0;
    nav_output.target_speed = 0;
    nav_output.switch_reason = NAV_SWITCH_NONE;
    nav_ctrl.last_steer_cmd = 0;
    nav_ctrl.last_camera_offset = 0;
    nav_ctrl.last_camera_slope = 0;
    nav_ctrl.last_camera_sample_valid = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     暂停导航
//-------------------------------------------------------------------------------------------------------------------
void nav_pause(void)
{
    if(nav_ctrl.state == NAV_STATE_RUNNING)
    {
        nav_ctrl.state = NAV_STATE_PAUSED;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     恢复导航
//-------------------------------------------------------------------------------------------------------------------
void nav_resume(void)
{
    if(nav_ctrl.state == NAV_STATE_PAUSED)
    {
        nav_ctrl.state = NAV_STATE_RUNNING;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航模式
// 参数说明     mode        NAV_MODE_CAMERA / NAV_MODE_GPS / NAV_MODE_MIXED
//-------------------------------------------------------------------------------------------------------------------
void nav_set_mode(uint8 mode)
{
    nav_ctrl.mode = mode;
}

//==================================================== 核心更新函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航主更新函数
// 参数说明     input       导航输入数据指针
// 返回参数     void
// 使用示例     nav_update(&nav_input);
// 备注信息     需要在主循环中周期性调用 (建议10-50ms)
//-------------------------------------------------------------------------------------------------------------------
void nav_update(nav_input_struct *input)
{
    float steer = 0;
    float target_speed = 0;
    uint8 camera_usable;
    uint8 gps_usable;
    uint8 source_available = 0;
    nav_task_enum selected_task;
    nav_source_enum selected_source;
    uint8 task_hold_limit;

    if(nav_ctrl.state != NAV_STATE_RUNNING)
    {
        return;
    }

    camera_usable = nav_camera_is_usable(input);
    gps_usable = nav_gps_is_usable(input);

    selected_task = nav_select_task(input, camera_usable, gps_usable);
    task_hold_limit = nav_get_task_hold_limit(selected_task);
    if(nav_should_switch_task_immediately(nav_ctrl.current_task, selected_task))
    {
        nav_ctrl.current_task = selected_task;
        nav_ctrl.task_hold_count = task_hold_limit;
    }
    else if(selected_task == nav_ctrl.current_task)
    {
        if(nav_ctrl.task_hold_count < task_hold_limit)
        {
            nav_ctrl.task_hold_count++;
        }
    }
    else if(nav_ctrl.task_hold_count > 0)
    {
        nav_ctrl.task_hold_count--;
    }
    else
    {
        nav_ctrl.current_task = selected_task;
        nav_ctrl.task_hold_count = task_hold_limit;
    }

    nav_output.camera_usable = camera_usable;
    nav_output.gps_usable = gps_usable;
    nav_output.current_task = nav_ctrl.current_task;
    nav_output.camera_age_ms = input->camera_update_age_ms;

    // 使用“可接管”条件而不是裸 camera_valid 做滞回判定
    nav_check_camera_status(camera_usable);

    // 只在GPS位置可用时更新路点跟踪状态
    if(input->gps_valid && input->gps_origin_ready && nav_path.count > 0)
    {
        nav_update_waypoint_tracking(input->pos_x, input->pos_y, nav_resolve_heading(input));
    }

    // 预计算两种导航源的原始输出，便于调试比较
    nav_output.raw_camera_steer = 0;
    nav_output.raw_gps_steer = 0;

    if(camera_usable)
    {
        nav_output.raw_camera_steer = nav_camera_to_steer(input->camera_center_offset,
                                                          input->camera_curvature);
    }

    if(gps_usable)
    {
        nav_output.raw_gps_steer = nav_gps_to_steer(input);
    }

    selected_source = nav_ctrl.preferred_source;

    switch(nav_ctrl.mode)
    {
        case NAV_MODE_CAMERA:
            if(camera_usable)
            {
                selected_source = NAV_SOURCE_CAMERA;
                source_available = 1;
                nav_output.switch_reason = NAV_SWITCH_FORCED_MODE;
            }
            else
            {
                nav_output.switch_reason = NAV_SWITCH_BOTH_UNAVAILABLE;
            }
            break;

        case NAV_MODE_GPS:
            if(gps_usable)
            {
                selected_source = NAV_SOURCE_GPS;
                source_available = 1;
                nav_output.switch_reason = NAV_SWITCH_FORCED_MODE;
            }
            else
            {
                nav_output.switch_reason = nav_get_gps_unusable_reason(input);
                if(nav_output.switch_reason == NAV_SWITCH_NONE)
                {
                    nav_output.switch_reason = NAV_SWITCH_BOTH_UNAVAILABLE;
                }
            }
            break;

        case NAV_MODE_MIXED:
        default:
            selected_source = nav_select_source(input, camera_usable, gps_usable);
            source_available = ((selected_source == NAV_SOURCE_CAMERA) && camera_usable) ||
                               ((selected_source == NAV_SOURCE_GPS) && gps_usable);
            break;
    }

    if(source_available)
    {
        nav_ctrl.preferred_source = selected_source;
        nav_ctrl.pending_source = selected_source;
        nav_output.active_source = selected_source;

        if(selected_source == NAV_SOURCE_CAMERA)
        {
            steer = nav_output.raw_camera_steer;
            nav_ctrl.camera_nav_time_ms++;
        }
        else
        {
            steer = nav_output.raw_gps_steer;
            nav_ctrl.gps_nav_time_ms++;
        }
    }
    else
    {
        steer = 0;
    }

    // 任务状态下的速度收敛
    switch(nav_ctrl.current_task)
    {
        case NAV_TASK_CONE_SLALOM:
            target_speed = 0.6f;
            break;
        case NAV_TASK_MINE_SEARCH:
            target_speed = 0.4f;
            break;
        case NAV_TASK_MINE_ROTATE:
            target_speed = 0.2f;
            break;
        case NAV_TASK_BRIDGE_PASS:
            target_speed = 0.35f;
            break;
        case NAV_TASK_STAIRS_PASS:
            target_speed = 0.3f;
            break;
        case NAV_TASK_RETURN_HOME:
            target_speed = 0.8f;
            break;
        case NAV_TASK_STRAIGHT_RUN:
        default:
            break;
    }

    // 设置目标速度（优先使用当前路点速度；纯视觉且无路点时给出默认速度）
    if(nav_path.current_index < nav_path.count)
    {
        target_speed = nav_path.points[nav_path.current_index].target_speed;
    }
    else if(source_available && selected_source == NAV_SOURCE_CAMERA)
    {
        target_speed = NAV_DEFAULT_CAMERA_SPEED;
    }

    if(!source_available)
    {
        target_speed = 0.3f;
    }

    nav_output.target_speed = target_speed;
    nav_output.bridge_execute = 0;
    nav_output.jump_execute = 0;
    nav_output.mine_rotate_execute = 0;

    if(nav_ctrl.current_task == NAV_TASK_BRIDGE_PASS)
    {
        if(input->bridge_alignment_ok && input->bridge_distance_level >= 2)
        {
            nav_output.bridge_execute = 1;
        }
    }
    else if(nav_ctrl.current_task == NAV_TASK_STAIRS_PASS)
    {
        if(input->stairs_alignment_ok && input->stairs_distance_level >= 3)
        {
            nav_output.jump_execute = 1;
        }
    }
    else if(nav_ctrl.current_task == NAV_TASK_MINE_ROTATE)
    {
        if(input->mine_box_detected && input->blue_zone_detected)
        {
            nav_output.mine_rotate_execute = 1;
            target_speed = 0.0f;
            steer = 18.0f;
        }
    }

    nav_output.steer_angle = nav_clamp(nav_apply_steer_filter(steer),
                                       -NAV_MAX_STEER_ANGLE,
                                       NAV_MAX_STEER_ANGLE);
    nav_output.camera_lost_frames = nav_ctrl.camera_lost_count;
    nav_output.camera_recover_frames = nav_ctrl.camera_recover_count;
    nav_output.source_hold_frames = nav_ctrl.source_hold_count;
    nav_output.task_hold_frames = nav_ctrl.task_hold_count;
    nav_output.state = nav_ctrl.state;
}

//==================================================== 航向计算函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算目标航向角
// 参数说明     cur_x, cur_y    当前位置
// 参数说明     tar_x, tar_y    目标位置
// 返回参数     float           目标航向角 (度, 北为0, 顺时针为正)
//-------------------------------------------------------------------------------------------------------------------
float nav_calc_target_heading(float cur_x, float cur_y, float tar_x, float tar_y)
{
    float dx = tar_x - cur_x;
    float dy = tar_y - cur_y;
    
    // atan2返回 -PI ~ PI，转换为 0~360 (北为0)
    float heading = atan2f(dx, dy) * NAV_RAD_TO_DEG;
    
    if(heading < 0)
    {
        heading += 360.0f;
    }
    
    return heading;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算航向偏差
// 参数说明     current_heading     当前航向 (度)
// 参数说明     target_heading      目标航向 (度)
// 返回参数     float               航向偏差 (度, 正值=需要右转)
//-------------------------------------------------------------------------------------------------------------------
float nav_calc_heading_error(float current_heading, float target_heading)
{
    float error = target_heading - current_heading;
    
    // 归一化到 -180 ~ 180
    error = nav_normalize_angle(error);
    
    return error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两点距离
// 参数说明     x1, y1      点1坐标
// 参数说明     x2, y2      点2坐标
// 返回参数     float       距离 (米)
//-------------------------------------------------------------------------------------------------------------------
float nav_calc_distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算横向偏差 (Cross Track Error)
// 参数说明     cur_x, cur_y    当前位置
// 参数说明     wp_index        目标路点索引
// 返回参数     float           横向偏差 (米, 正值=在路径右侧)
// 备注信息     计算当前位置到前一路点->当前路点连线的垂直距离
//-------------------------------------------------------------------------------------------------------------------
float nav_calc_cross_track_error(float cur_x, float cur_y, uint16 wp_index)
{
    if(wp_index == 0 || wp_index >= nav_path.count)
    {
        return 0;
    }
    
    // 前一个路点
    float x1 = nav_path.points[wp_index - 1].x;
    float y1 = nav_path.points[wp_index - 1].y;
    
    // 当前目标路点
    float x2 = nav_path.points[wp_index].x;
    float y2 = nav_path.points[wp_index].y;
    
    // 路径向量
    float dx = x2 - x1;
    float dy = y2 - y1;
    float path_len = sqrtf(dx * dx + dy * dy);
    
    if(path_len < 0.001f)
    {
        return 0;
    }
    
    // 计算叉积得到横向偏差
    float cross = (cur_x - x1) * dy - (cur_y - y1) * dx;
    
    return cross / path_len;
}

//==================================================== Pure Pursuit 算法 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Pure Pursuit 路径跟踪算法
// 参数说明     cur_x, cur_y    当前位置 (米)
// 参数说明     cur_heading     当前航向 (度, 北为0, 顺时针为正)
// 参数说明     lookahead       前瞻距离 (米)
// 返回参数     float           目标转向角 (度)
// 备注信息     经典的路径跟踪算法，适合车辆低速跟踪
//-------------------------------------------------------------------------------------------------------------------
float nav_pure_pursuit(float cur_x, float cur_y, float cur_heading, float lookahead)
{
    if(nav_path.count == 0 || nav_path.current_index >= nav_path.count)
    {
        return 0;
    }
    
    // 获取目标路点
    float target_x = nav_path.points[nav_path.current_index].x;
    float target_y = nav_path.points[nav_path.current_index].y;
    
    // 计算到目标的方位角 (北为0, 顺时针为正)
    float dx = target_x - cur_x;
    float dy = target_y - cur_y;
    float target_heading = atan2f(dx, dy) * NAV_RAD_TO_DEG;
    if(target_heading < 0) target_heading += 360.0f;
    
    // 计算航向偏差
    float heading_error = target_heading - cur_heading;
    
    // 归一化到 -180 ~ 180
    while(heading_error > 180.0f) heading_error -= 360.0f;
    while(heading_error < -180.0f) heading_error += 360.0f;
    
    // 简单比例控制：航向偏差直接映射到转向角
    // Kp = 0.5 表示 60度偏差 -> 30度转向
    float Kp = 0.5f;
    float steer_deg = heading_error * Kp;
    
    // 保存调试信息
    float dist = sqrtf(dx * dx + dy * dy);
    nav_output.distance_to_target = dist;
    nav_output.target_heading = target_heading;
    nav_output.heading_error = heading_error;
    
    return steer_deg;
}

//==================================================== 模式切换函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查摄像头状态，自动切换导航模式
// 参数说明     camera_valid    摄像头数据是否有效 (0=丢线)
// 返回参数     void
// 备注信息     使用滞回逻辑防止频繁切换
//-------------------------------------------------------------------------------------------------------------------
void nav_check_camera_status(uint8 camera_valid)
{
    if(!camera_valid)
    {
        // 摄像头丢线
        nav_ctrl.camera_lost_count++;
        nav_ctrl.camera_recover_count = 0;
        
        if(nav_ctrl.camera_lost_count >= NAV_CAMERA_LOST_THRESHOLD)
        {
            nav_ctrl.is_camera_lost = 1;        // 确认丢线，切换到GPS
        }
    }
    else
    {
        // 摄像头恢复
        nav_ctrl.camera_recover_count++;
        nav_ctrl.camera_lost_count = 0;
        
        if(nav_ctrl.camera_recover_count >= NAV_CAMERA_RECOVER_THRESHOLD)
        {
            nav_ctrl.is_camera_lost = 0;        // 确认恢复，切换回摄像头
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前激活的数据源
// 返回参数     nav_source_enum     NAV_SOURCE_CAMERA / NAV_SOURCE_GPS
//-------------------------------------------------------------------------------------------------------------------
nav_source_enum nav_get_active_source(void)
{
    return nav_output.active_source;
}

//==================================================== 内部函数实现 ====================================================

static uint8 nav_camera_is_usable(const nav_input_struct *input)
{
    int16 offset_jump;
    int16 slope_jump;

    if(!input->camera_valid)
    {
        nav_ctrl.last_camera_sample_valid = 0;
        return 0;
    }

    if(input->camera_valid_rows < NAV_CAMERA_MIN_VALID_ROWS)
    {
        nav_ctrl.last_camera_sample_valid = 0;
        return 0;
    }

    if(input->camera_update_age_ms > NAV_CAMERA_MAX_AGE_MS)
    {
        nav_ctrl.last_camera_sample_valid = 0;
        return 0;
    }

    if(input->camera_road_type == NAV_CAMERA_ROAD_LOST_TYPE)
    {
        nav_ctrl.last_camera_sample_valid = 0;
        return 0;
    }

    if(!nav_ctrl.last_camera_sample_valid)
    {
        nav_ctrl.last_camera_offset = input->camera_center_offset;
        nav_ctrl.last_camera_slope = input->camera_center_slope;
        nav_ctrl.last_camera_sample_valid = 1;
        return 1;
    }

    offset_jump = input->camera_center_offset - nav_ctrl.last_camera_offset;
    if(offset_jump < 0)
    {
        offset_jump = -offset_jump;
    }

    slope_jump = input->camera_center_slope - nav_ctrl.last_camera_slope;
    if(slope_jump < 0)
    {
        slope_jump = -slope_jump;
    }

    nav_ctrl.last_camera_offset = input->camera_center_offset;
    nav_ctrl.last_camera_slope = input->camera_center_slope;

    if(offset_jump > NAV_CAMERA_OFFSET_JUMP_MAX)
    {
        return 0;
    }

    if(slope_jump > NAV_CAMERA_SLOPE_JUMP_MAX)
    {
        return 0;
    }

    return 1;
}

static uint8 nav_gps_is_usable(const nav_input_struct *input)
{
    return (NAV_SWITCH_NONE == nav_get_gps_unusable_reason(input));
}

static nav_switch_reason_enum nav_get_gps_unusable_reason(const nav_input_struct *input)
{
    if(!input->gps_valid || !input->gps_reliable || !input->gps_origin_ready)
    {
        return NAV_SWITCH_GPS_UNRELIABLE;
    }

    if(input->gps_hdop > NAV_GPS_HDOP_ACCEPT)
    {
        return NAV_SWITCH_GPS_UNRELIABLE;
    }

    if(input->gps_update_age_ms > NAV_GPS_MAX_AGE_MS)
    {
        return NAV_SWITCH_GPS_STALE;
    }

    if(nav_path.count == 0 || nav_path.current_index >= nav_path.count)
    {
        return NAV_SWITCH_NO_WAYPOINTS;
    }

    return NAV_SWITCH_NONE;
}

static nav_switch_reason_enum nav_get_hold_reason(const nav_input_struct *input, nav_source_enum source, uint8 camera_usable, uint8 gps_usable)
{
    nav_switch_reason_enum gps_reason = nav_get_gps_unusable_reason(input);

    if(!camera_usable && !gps_usable)
    {
        if(gps_reason != NAV_SWITCH_NONE)
        {
            return gps_reason;
        }
        return NAV_SWITCH_BOTH_UNAVAILABLE;
    }

    if(source == NAV_SOURCE_CAMERA)
    {
        if(!gps_usable)
        {
            if(gps_reason != NAV_SWITCH_NONE)
            {
                return gps_reason;
            }
            return NAV_SWITCH_GPS_UNRELIABLE;
        }
        return NAV_SWITCH_CAMERA_TRACKING;
    }

    if(source == NAV_SOURCE_GPS)
    {
        if(!camera_usable || nav_ctrl.is_camera_lost)
        {
            return NAV_SWITCH_CAMERA_LOST;
        }
        return NAV_SWITCH_GPS_TRACKING;
    }

    return NAV_SWITCH_NONE;
}

static nav_task_enum nav_select_task(const nav_input_struct *input, uint8 camera_usable, uint8 gps_usable)
{
    if(nav_ctrl.current_task == NAV_TASK_MINE_SEARCH)
    {
        if(input->mine_box_detected)
        {
            if(input->blue_zone_detected && input->blue_zone_hold_frames >= 2)
            {
                return NAV_TASK_MINE_ROTATE;
            }
            return NAV_TASK_MINE_SEARCH;
        }
    }

    if(nav_ctrl.current_task == NAV_TASK_CONE_SLALOM)
    {
        if(input->cone_gap_detected || input->cone_gap_hold_frames > 0)
        {
            return NAV_TASK_CONE_SLALOM;
        }
    }

    if(nav_ctrl.current_task == NAV_TASK_STAIRS_PASS)
    {
        if(input->stairs_detected || input->blue_zone_hold_frames > 0)
        {
            return NAV_TASK_STAIRS_PASS;
        }
    }

    if(nav_ctrl.current_task == NAV_TASK_BRIDGE_PASS)
    {
        if(input->bridge_detected || input->bridge_hold_frames > 0)
        {
            return NAV_TASK_BRIDGE_PASS;
        }
    }

    if(input->mine_box_detected)
    {
        if(input->blue_zone_detected && input->blue_zone_hold_frames >= 2)
        {
            return NAV_TASK_MINE_ROTATE;
        }
        return NAV_TASK_MINE_SEARCH;
    }

    if(input->bridge_detected && input->bridge_hold_frames >= 2)
    {
        return NAV_TASK_BRIDGE_PASS;
    }

    if((input->stairs_detected && input->stairs_hold_frames >= 2) ||
       (input->blue_zone_detected && input->blue_zone_hold_frames >= 2))
    {
        return NAV_TASK_STAIRS_PASS;
    }

    if(input->cone_gap_detected && input->cone_gap_hold_frames >= 1)
    {
        return NAV_TASK_CONE_SLALOM;
    }

    if(gps_usable)
    {
        return NAV_TASK_RETURN_HOME;
    }

    if(camera_usable)
    {
        return NAV_TASK_STRAIGHT_RUN;
    }

    return NAV_TASK_IDLE;
}

static uint8 nav_get_task_hold_limit(nav_task_enum task)
{
    switch(task)
    {
        case NAV_TASK_MINE_SEARCH:
        case NAV_TASK_MINE_ROTATE:
        case NAV_TASK_BRIDGE_PASS:
            return 2;
        case NAV_TASK_STAIRS_PASS:
        case NAV_TASK_CONE_SLALOM:
            return 1;
        case NAV_TASK_RETURN_HOME:
        case NAV_TASK_STRAIGHT_RUN:
        case NAV_TASK_IDLE:
        default:
            return 0;
    }
}

static uint8 nav_should_switch_task_immediately(nav_task_enum current_task, nav_task_enum selected_task)
{
    if(current_task == NAV_TASK_MINE_SEARCH && selected_task == NAV_TASK_MINE_ROTATE)
    {
        return 1;
    }

    if(current_task == NAV_TASK_RETURN_HOME && selected_task == NAV_TASK_STRAIGHT_RUN)
    {
        return 1;
    }

    return 0;
}

static nav_source_enum nav_select_source(const nav_input_struct *input, uint8 camera_usable, uint8 gps_usable)
{
    (void)input;

    if(nav_ctrl.current_task == NAV_TASK_CONE_SLALOM ||
       nav_ctrl.current_task == NAV_TASK_MINE_SEARCH ||
       nav_ctrl.current_task == NAV_TASK_MINE_ROTATE ||
       nav_ctrl.current_task == NAV_TASK_BRIDGE_PASS ||
       nav_ctrl.current_task == NAV_TASK_STAIRS_PASS)
    {
        if(camera_usable)
        {
            if(nav_ctrl.preferred_source != NAV_SOURCE_CAMERA)
            {
                nav_output.switch_reason = NAV_SWITCH_CAMERA_RECOVERED;
                nav_ctrl.source_hold_count = 0;
            }
            else
            {
                nav_output.switch_reason = NAV_SWITCH_CAMERA_TRACKING;
                nav_ctrl.source_hold_count++;
            }
            return NAV_SOURCE_CAMERA;
        }

        nav_output.switch_reason = NAV_SWITCH_BOTH_UNAVAILABLE;
        return nav_ctrl.preferred_source;
    }

    if(camera_usable && !nav_ctrl.is_camera_lost)
    {
        if(nav_ctrl.preferred_source == NAV_SOURCE_GPS && gps_usable &&
           nav_ctrl.source_hold_count < NAV_SOURCE_MIN_HOLD_CYCLES)
        {
            nav_output.switch_reason = NAV_SWITCH_GPS_TRACKING;
            nav_ctrl.source_hold_count++;
            return NAV_SOURCE_GPS;
        }

        if(nav_ctrl.preferred_source != NAV_SOURCE_CAMERA)
        {
            nav_output.switch_reason = NAV_SWITCH_CAMERA_RECOVERED;
            nav_ctrl.source_hold_count = 0;
        }
        else
        {
            nav_output.switch_reason = nav_get_hold_reason(input, NAV_SOURCE_CAMERA, camera_usable, gps_usable);
            nav_ctrl.source_hold_count++;
        }
        return NAV_SOURCE_CAMERA;
    }

    if(gps_usable)
    {
        if(nav_ctrl.preferred_source != NAV_SOURCE_GPS)
        {
            nav_output.switch_reason = NAV_SWITCH_CAMERA_LOST;
            nav_ctrl.source_hold_count = 0;
        }
        else
        {
            nav_output.switch_reason = nav_get_hold_reason(input, NAV_SOURCE_GPS, camera_usable, gps_usable);
            nav_ctrl.source_hold_count++;
        }
        return NAV_SOURCE_GPS;
    }

    if(camera_usable)
    {
        if(nav_ctrl.preferred_source != NAV_SOURCE_CAMERA)
        {
            nav_output.switch_reason = nav_get_gps_unusable_reason(input);
            if(nav_output.switch_reason == NAV_SWITCH_NONE)
            {
                nav_output.switch_reason = NAV_SWITCH_GPS_UNRELIABLE;
            }
            nav_ctrl.source_hold_count = 0;
        }
        else
        {
            if(nav_get_gps_unusable_reason(input) != NAV_SWITCH_NONE)
            {
                nav_output.switch_reason = nav_get_gps_unusable_reason(input);
            }
            else
            {
                nav_output.switch_reason = NAV_SWITCH_CAMERA_TRACKING;
            }
            nav_ctrl.source_hold_count++;
        }
        return NAV_SOURCE_CAMERA;
    }

    if(nav_get_gps_unusable_reason(input) != NAV_SWITCH_NONE)
    {
        nav_output.switch_reason = nav_get_gps_unusable_reason(input);
    }
    else
    {
        nav_output.switch_reason = NAV_SWITCH_BOTH_UNAVAILABLE;
    }
    return nav_ctrl.preferred_source;
}

static float nav_apply_steer_filter(float steer)
{
    float delta = steer - nav_ctrl.last_steer_cmd;

    delta = nav_clamp(delta, -NAV_STEER_SLEW_LIMIT, NAV_STEER_SLEW_LIMIT);
    nav_ctrl.last_steer_cmd += delta;

    return nav_ctrl.last_steer_cmd;
}

static float nav_resolve_heading(const nav_input_struct *input)
{
    if(input->current_speed < NAV_GPS_HEADING_MIN_SPEED)
    {
        if(input->imu_yaw > -360.0f && input->imu_yaw < 360.0f && input->imu_yaw != 0.0f)
        {
            nav_output.used_heading = input->imu_yaw;
            return input->imu_yaw;
        }
    }

    nav_output.used_heading = input->gps_heading;
    return input->gps_heading;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头数据转转向角
// 参数说明     center_offset   中线偏移 (像素)
// 参数说明     curvature       曲率估计
// 返回参数     float           转向角 (度)
//-------------------------------------------------------------------------------------------------------------------
static float nav_camera_to_steer(int16 center_offset, float curvature)
{
    float steer = (float)center_offset * 0.2f;

    switch(nav_ctrl.current_task)
    {
        case NAV_TASK_CONE_SLALOM:
            steer = (float)center_offset * 0.30f + curvature * 6.0f;
            break;
        case NAV_TASK_MINE_SEARCH:
            steer = (float)center_offset * 0.18f + curvature * 3.0f;
            break;
        case NAV_TASK_MINE_ROTATE:
            steer = (float)center_offset * 0.10f;
            break;
        case NAV_TASK_BRIDGE_PASS:
            steer = (float)center_offset * 0.14f + curvature * 2.0f;
            break;
        case NAV_TASK_STAIRS_PASS:
            steer = (float)center_offset * 0.12f + curvature * 1.5f;
            break;
        case NAV_TASK_STRAIGHT_RUN:
        case NAV_TASK_RETURN_HOME:
        default:
            steer = (float)center_offset * 0.2f + curvature * 5.0f;
            break;
    }

    return steer;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS数据转转向角 (使用Pure Pursuit)
// 参数说明     input       导航输入数据
// 返回参数     float       转向角 (度)
//-------------------------------------------------------------------------------------------------------------------
static float nav_gps_to_steer(nav_input_struct *input)
{
    float heading = nav_resolve_heading(input);

    // 使用Pure Pursuit算法（已简化为航向跟踪）
    float steer = nav_pure_pursuit(input->pos_x, input->pos_y,
                                   heading, NAV_LOOKAHEAD_DIST);
    
    // 保存横向偏差信息
    nav_output.cross_track_error = nav_calc_cross_track_error(input->pos_x, input->pos_y, 
                                                               nav_path.current_index);
    
    return steer;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新路点跟踪状态
// 参数说明     cur_x, cur_y    当前位置
// 备注信息     检查是否到达当前路点，切换到下一个
//-------------------------------------------------------------------------------------------------------------------
static void nav_update_waypoint_tracking(float cur_x, float cur_y, float heading)
{
    if(nav_path.current_index >= nav_path.count)
    {
        return;
    }
    
    waypoint_struct *target = &nav_path.points[nav_path.current_index];
    float dist = nav_calc_distance(cur_x, cur_y, target->x, target->y);
    
    nav_output.distance_to_target = dist;
    
    // 计算航向偏差
    float target_heading = nav_calc_target_heading(cur_x, cur_y, target->x, target->y);
    nav_output.target_heading = target_heading;
    nav_output.heading_error = nav_calc_heading_error(heading, target_heading);
    
    // 检查是否到达路点
    if(dist < NAV_WAYPOINT_REACH_DIST)
    {
        nav_path.current_index++;
        
        // 检查是否到达终点
        if(nav_path.current_index >= nav_path.count)
        {
            if(nav_path.is_loop)
            {
                nav_path.current_index = 0;     // 循环
            }
            else
            {
                nav_ctrl.state = NAV_STATE_REACHED;
            }
        }
    }
}

//==================================================== 辅助函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度归一化到 -180 ~ 180
//-------------------------------------------------------------------------------------------------------------------
float nav_normalize_angle(float angle)
{
    while(angle > 180.0f)  angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数值限幅
//-------------------------------------------------------------------------------------------------------------------
float nav_clamp(float value, float min, float max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}
