/*
 * navigation_bridge.c
 *
 * CM7_0 侧桥接层：
 *   1. 组装 nav_input_struct
 *   2. 调用 nav_update()
 *   3. 将 nav_output 编码为 IPC 控制命令发送给 CM7_1
 */

#include "navigation_bridge.h"

#include "camera_service.h"
#include "gps_service.h"
#include "nav_decision.h"
#include "IMU.h"

static nav_input_struct s_nav_input;
static nav_bridge_debug_struct s_nav_bridge_debug;
static uint32 s_last_camera_frame_ms = 0;
static uint32 s_last_gps_update_ms = 0;
static uint8 s_gps_origin_auto_set = 0;
static uint8 s_nav_started = 0;

static uint32 nav_bridge_make_packet(uint8 cmd, int16 value)
{
    return (((uint32)cmd) << 24) | ((uint16)value);
}

static void nav_bridge_send_targets(void)
{
    float speed_value = nav_output.target_speed * NAV_BRIDGE_SPEED_SCALE;
    float steer_delta = nav_output.steer_angle * NAV_BRIDGE_STEER_SCALE;
    uint8 flags = 0;
    int16 speed_cmd;
    int16 turn_cmd;

    if(speed_value > NAV_BRIDGE_SPEED_MAX) speed_value = NAV_BRIDGE_SPEED_MAX;
    if(speed_value < NAV_BRIDGE_SPEED_MIN) speed_value = NAV_BRIDGE_SPEED_MIN;

    if(steer_delta > NAV_BRIDGE_STEER_MAX) steer_delta = NAV_BRIDGE_STEER_MAX;
    if(steer_delta < -NAV_BRIDGE_STEER_MAX) steer_delta = -NAV_BRIDGE_STEER_MAX;

    speed_cmd = (int16)speed_value;
    turn_cmd = (int16)(steer_delta * NAV_BRIDGE_STEER_PACKET_SCALE);

    s_nav_bridge_debug.speed_cmd = speed_cmd;
    s_nav_bridge_debug.turn_cmd = turn_cmd;

    if(nav_ctrl.state == NAV_STATE_RUNNING)
    {
        flags |= CHASSIS_IPC_FLAG_TURN_EN;
    }

    if(nav_output.current_task == NAV_TASK_BRIDGE_PASS)
    {
        flags |= CHASSIS_IPC_FLAG_BRIDGE_EN;
    }
    else if(nav_output.current_task == NAV_TASK_STAIRS_PASS)
    {
        flags |= CHASSIS_IPC_FLAG_JUMP_EN;
    }

    s_nav_bridge_debug.flags = flags;

    ipc_send_data(nav_bridge_make_packet(CHASSIS_IPC_CMD_SPEED, speed_cmd));
    ipc_send_data(nav_bridge_make_packet(CHASSIS_IPC_CMD_TURN, turn_cmd));
    ipc_send_data(nav_bridge_make_packet(CHASSIS_IPC_CMD_FLAGS, (int16)flags));
}

void nav_bridge_init(void)
{
    memset(&s_nav_input, 0, sizeof(s_nav_input));
    s_last_camera_frame_ms = 0;
    s_last_gps_update_ms = 0;
    s_gps_origin_auto_set = 0;
    s_nav_started = 0;

    nav_decision_init();
    nav_set_mode(NAV_MODE_CAMERA);
}

void nav_bridge_notify_camera_frame(uint32 now_ms)
{
    s_last_camera_frame_ms = now_ms;
}

void nav_bridge_notify_gps_update(uint32 now_ms)
{
    s_last_gps_update_ms = now_ms;
}

const nav_bridge_debug_struct* nav_bridge_get_debug(void)
{
    return &s_nav_bridge_debug;
}

void nav_bridge_update(uint32 now_ms)
{
    s_nav_input.pos_x = gps_data.pos_x;
    s_nav_input.pos_y = gps_data.pos_y;
    s_nav_input.gps_heading = gps_data.course;
    s_nav_input.gps_valid = gps_is_fixed();
    s_nav_input.gps_reliable = gps_is_reliable();
    s_nav_input.gps_origin_ready = gps_data.origin_set;
    s_nav_input.gps_hdop = gps_data.hdop;
    s_nav_input.gps_update_age_ms = (now_ms >= s_last_gps_update_ms) ? (now_ms - s_last_gps_update_ms) : 0;

    s_nav_input.camera_center_offset = track_info.offset;
    s_nav_input.camera_curvature = (float)track_info.center_slope / 100.0f;
    s_nav_input.camera_valid = track_info.is_valid;
    s_nav_input.camera_valid_rows = track_info.valid_row_count;
    s_nav_input.camera_center_slope = track_info.center_slope;
    s_nav_input.camera_road_type = (uint8)track_info.road_type;
    s_nav_input.camera_update_age_ms = (s_last_camera_frame_ms > 0 && now_ms >= s_last_camera_frame_ms) ?
                                       (now_ms - s_last_camera_frame_ms) : 10000;

    s_nav_input.current_speed = gps_data.speed / 3.6f;
    s_nav_input.imu_yaw = (float)eulerAngle.yaw;

    s_nav_input.blue_zone_detected = camera_task_info.blue_zone_detected;
    s_nav_input.blue_zone_hold_frames = camera_task_info.blue_zone_hold_frames;
    s_nav_input.mine_box_detected = camera_task_info.mine_box_detected;
    s_nav_input.mine_box_hold_frames = camera_task_info.mine_box_hold_frames;
    s_nav_input.mine_box_offset = camera_task_info.mine_box_offset;
    s_nav_input.bridge_detected = camera_task_info.bridge_detected;
    s_nav_input.bridge_hold_frames = camera_task_info.bridge_hold_frames;
    s_nav_input.bridge_center_offset = camera_task_info.bridge_center_offset;
    s_nav_input.stairs_detected = camera_task_info.stairs_detected;
    s_nav_input.stairs_hold_frames = camera_task_info.stairs_hold_frames;
    s_nav_input.stairs_center_offset = camera_task_info.stairs_center_offset;
    s_nav_input.cone_gap_detected = camera_task_info.cone_gap_detected;
    s_nav_input.cone_gap_hold_frames = camera_task_info.cone_gap_hold_frames;
    s_nav_input.cone_gap_offset = camera_task_info.cone_gap_offset;

    if(!s_gps_origin_auto_set && gps_data.status != GPS_STATUS_INVALID && gps_data.is_reliable)
    {
        if(gps_set_origin())
        {
            s_gps_origin_auto_set = 1;
        }
    }

    if(!s_nav_started)
    {
        if(gps_data.origin_set && nav_get_waypoint_count() > 0)
        {
            nav_start();
            s_nav_started = 1;
        }
        else if(nav_get_waypoint_count() == 0 && track_info.is_valid)
        {
            nav_start();
            s_nav_started = 1;
        }
    }

    nav_update(&s_nav_input);

    if(nav_ctrl.state != NAV_STATE_RUNNING)
    {
        ipc_send_data(nav_bridge_make_packet(CHASSIS_IPC_CMD_SPEED, 0));
        ipc_send_data(nav_bridge_make_packet(CHASSIS_IPC_CMD_TURN, 0));
        ipc_send_data(nav_bridge_make_packet(CHASSIS_IPC_CMD_FLAGS, 0));
        return;
    }

    nav_bridge_send_targets();
}
