/*********************************************************************************************************************
* CM7_0 上层感知与决策核心
* - 摄像头 / GPS / 决策模块运行在主循环低频任务中
* - 通过 navigation_bridge 将 nav_output 转成 IPC 控制命令发送给 CM7_1
*********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "../code/camera_module/camera_service.h"
#include "../code/gps_module/gps_service.h"
#include "../code/decision_module/nav_decision.h"
#include "../code/IMU.h"
#include "../code/navigation_bridge.h"

#define LED1                          (P19_0)
#define CAMERA_STREAM_LOG_INTERVAL_MS  (500u)
#define WIFI_CAMERA_SSID               "YOUR_WIFI_SSID"
#define WIFI_CAMERA_PASSWORD           "YOUR_WIFI_PASSWORD"
#define WIFI_CAMERA_TARGET_IP          "10.7.7.150"
#define WIFI_CAMERA_TARGET_PORT        "8086"
#define WIFI_CAMERA_LOCAL_PORT         "6666"
#define ASSISTANT_OSC_CHANNEL_COUNT    (8u)

static void ipc_dummy_callback(uint32 ipc_data)
{
    (void)ipc_data;
}

void gps_debug_callback(void)
{
}

static uint8 assistant_center_line[CAMERA_IMAGE_H];
static uint8 assistant_bridge_line[CAMERA_IMAGE_H];
static uint8 assistant_stairs_line[CAMERA_IMAGE_H];

static uint8 obstacle_code(void)
{
    if(camera_task_info.bridge_detected)
    {
        return 1;
    }
    if(camera_task_info.stairs_detected)
    {
        return 2;
    }
    if(camera_task_info.mine_box_detected)
    {
        return 3;
    }
    if(camera_task_info.cone_gap_detected)
    {
        return 4;
    }
    return 0;
}

static void assistant_send_telemetry(void)
{
    seekfree_assistant_oscilloscope_data.channel_num = ASSISTANT_OSC_CHANNEL_COUNT;
    seekfree_assistant_oscilloscope_data.data[0] = (float)track_info.offset;
    seekfree_assistant_oscilloscope_data.data[1] = (float)track_info.center_slope / 100.0f;
    seekfree_assistant_oscilloscope_data.data[2] = camera_task_info.bridge_detected ? camera_task_info.bridge_offset_angle_deg : 0.0f;
    seekfree_assistant_oscilloscope_data.data[3] = camera_task_info.bridge_detected ? camera_task_info.bridge_forward_distance_cm : 0.0f;
    seekfree_assistant_oscilloscope_data.data[4] = camera_task_info.stairs_detected ? camera_task_info.stairs_offset_angle_deg : 0.0f;
    seekfree_assistant_oscilloscope_data.data[5] = camera_task_info.stairs_detected ? camera_task_info.stairs_forward_distance_cm : 0.0f;
    seekfree_assistant_oscilloscope_data.data[6] = (float)obstacle_code();
    seekfree_assistant_oscilloscope_data.data[7] = (float)nav_output.current_task;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

static const char* obstacle_name(void)
{
    if(camera_task_info.bridge_detected)
    {
        return "BRIDGE";
    }
    if(camera_task_info.stairs_detected)
    {
        return "STAIRS";
    }
    if(camera_task_info.mine_box_detected)
    {
        return "MINE";
    }
    if(camera_task_info.cone_gap_detected)
    {
        return "CONE";
    }
    return "NONE";
}

static uint8 clamp_image_x(int16 x)
{
    if(x < 0)
    {
        return 0;
    }
    if(x >= CAMERA_IMAGE_W)
    {
        return CAMERA_IMAGE_W - 1;
    }
    return (uint8)x;
}

static void fill_vertical_overlay(uint8 *buffer, uint8 x)
{
    uint16 row;

    for(row = 0; row < CAMERA_IMAGE_H; row++)
    {
        buffer[row] = x;
    }
}

static void update_camera_overlay(void)
{
    uint16 row;
    uint8 bridge_x = clamp_image_x(CAMERA_IMAGE_CENTER + camera_task_info.bridge_center_offset);
    uint8 stairs_x = clamp_image_x(CAMERA_IMAGE_CENTER + camera_task_info.stairs_center_offset);

    for(row = 0; row < CAMERA_IMAGE_H; row++)
    {
        if(row >= CAMERA_SCAN_END_ROW)
        {
            assistant_center_line[row] = track_info.center_line[row];
        }
        else
        {
            assistant_center_line[row] = CAMERA_IMAGE_CENTER;
        }
    }

    fill_vertical_overlay(assistant_bridge_line, bridge_x);
    fill_vertical_overlay(assistant_stairs_line, stairs_x);

    if(camera_task_info.bridge_detected && camera_task_info.stairs_detected)
    {
        seekfree_assistant_camera_boundary_config(X_BOUNDARY, CAMERA_IMAGE_H,
                                                  assistant_center_line,
                                                  assistant_bridge_line,
                                                  assistant_stairs_line,
                                                  NULL, NULL, NULL);
    }
    else if(camera_task_info.bridge_detected)
    {
        seekfree_assistant_camera_boundary_config(X_BOUNDARY, CAMERA_IMAGE_H,
                                                  assistant_center_line,
                                                  assistant_bridge_line,
                                                  NULL,
                                                  NULL, NULL, NULL);
    }
    else if(camera_task_info.stairs_detected)
    {
        seekfree_assistant_camera_boundary_config(X_BOUNDARY, CAMERA_IMAGE_H,
                                                  assistant_center_line,
                                                  assistant_stairs_line,
                                                  NULL,
                                                  NULL, NULL, NULL);
    }
    else
    {
        seekfree_assistant_camera_boundary_config(X_BOUNDARY, CAMERA_IMAGE_H,
                                                  assistant_center_line,
                                                  NULL,
                                                  NULL,
                                                  NULL, NULL, NULL);
    }
}

static const char* road_type_name(uint8 road_type)
{
    switch(road_type)
    {
        case ROAD_STRAIGHT: return "STRAIGHT";
        case ROAD_CURVE_L:  return "CURVE_L";
        case ROAD_CURVE_R:  return "CURVE_R";
        case ROAD_LOST:     return "LOST";
        case ROAD_OBSTACLE: return "OBSTACLE";
        case ROAD_CROSS:    return "CROSS";
        default:            return "UNKNOWN";
    }
}

static const char* nav_source_name(nav_source_enum source)
{
    switch(source)
    {
        case NAV_SOURCE_CAMERA: return "CAM";
        case NAV_SOURCE_GPS:    return "GPS";
        case NAV_SOURCE_FUSED:  return "FUSED";
        default:                return "UNKNOWN";
    }
}

static const char* nav_state_name(nav_state_enum state)
{
    switch(state)
    {
        case NAV_STATE_IDLE:    return "IDLE";
        case NAV_STATE_RUNNING: return "RUN";
        case NAV_STATE_PAUSED:  return "PAUSE";
        case NAV_STATE_REACHED: return "REACHED";
        case NAV_STATE_ERROR:   return "ERROR";
        default:                return "UNKNOWN";
    }
}

static const char* nav_task_name(nav_task_enum task)
{
    switch(task)
    {
        case NAV_TASK_IDLE:         return "IDLE";
        case NAV_TASK_STRAIGHT_RUN: return "STRAIGHT";
        case NAV_TASK_CONE_SLALOM:  return "CONE";
        case NAV_TASK_MINE_SEARCH:  return "MINE_SEARCH";
        case NAV_TASK_MINE_ROTATE:  return "MINE_ROTATE";
        case NAV_TASK_BRIDGE_PASS:  return "BRIDGE";
        case NAV_TASK_STAIRS_PASS:  return "STAIRS";
        case NAV_TASK_RETURN_HOME:  return "RETURN";
        default:                    return "UNKNOWN";
    }
}

static const char* nav_switch_reason_name(nav_switch_reason_enum reason)
{
    switch(reason)
    {
        case NAV_SWITCH_NONE:             return "NONE";
        case NAV_SWITCH_CAMERA_TRACKING:  return "CAM_TRACK";
        case NAV_SWITCH_GPS_TRACKING:     return "GPS_TRACK";
        case NAV_SWITCH_CAMERA_LOST:      return "CAM_LOST";
        case NAV_SWITCH_CAMERA_RECOVERED: return "CAM_RECOVER";
        case NAV_SWITCH_GPS_UNRELIABLE:   return "GPS_BAD";
        case NAV_SWITCH_GPS_STALE:        return "GPS_STALE";
        case NAV_SWITCH_NO_WAYPOINTS:     return "NO_WP";
        case NAV_SWITCH_BOTH_UNAVAILABLE: return "NO_SRC";
        case NAV_SWITCH_FORCED_MODE:      return "FORCED";
        default:                          return "UNKNOWN";
    }
}

static void print_obstacle_validation_log(const nav_bridge_debug_struct *bridge_debug)
{
    float obs_angle = 0.0f;
    float obs_dist = 0.0f;

    if(camera_task_info.bridge_detected)
    {
        obs_angle = camera_task_info.bridge_offset_angle_deg;
        obs_dist = camera_task_info.bridge_forward_distance_cm;
    }
    else if(camera_task_info.stairs_detected)
    {
        obs_angle = camera_task_info.stairs_offset_angle_deg;
        obs_dist = camera_task_info.stairs_forward_distance_cm;
    }

    printf("[TEL] road:%s src:%s task:%s sw:%s | mid:%d slope:%.2f rows:%u | obs:%s ang:%.2f dist:%.1fcm | spd:%d turn:%d flags:0x%02X\r\n",
           road_type_name(track_info.road_type),
           nav_source_name(nav_output.active_source),
           nav_task_name(nav_output.current_task),
           nav_switch_reason_name(nav_output.switch_reason),
           track_info.offset,
           (float)track_info.center_slope / 100.0f,
           track_info.valid_row_count,
           obstacle_name(),
           obs_angle,
           obs_dist,
           bridge_debug ? bridge_debug->speed_cmd : 0,
           bridge_debug ? bridge_debug->turn_cmd : 0,
           bridge_debug ? bridge_debug->flags : 0);
}

int main(void)
{
    uint8 camera_init_ok = 0;
    uint8 gps_init_ok = 0;
    uint8 wifi_init_ok = 0;
    uint8 socket_init_ok = 0;
    uint8 gps_updated = 0;
    uint8 frame_updated = 0;
    uint32 last_log_ms = 0;
    const nav_bridge_debug_struct *bridge_debug;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    IMU_init();

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);

    ipc_communicate_init(IPC_PORT_1, ipc_dummy_callback);
    nav_bridge_init();

    camera_init_ok = (camera_service_init() == 0);
    gps_service_init();
    gps_init_ok = 1;

    if(camera_init_ok)
    {
        wifi_init_ok = (wifi_spi_init(WIFI_CAMERA_SSID, WIFI_CAMERA_PASSWORD) == 0);
        if(wifi_init_ok)
        {
            socket_init_ok = (wifi_spi_socket_connect("UDP", WIFI_CAMERA_TARGET_IP, WIFI_CAMERA_TARGET_PORT, WIFI_CAMERA_LOCAL_PORT) == 0);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        mt9v03x_image[0],
        MT9V03X_W,
        MT9V03X_H
    );
    update_camera_overlay();

    timer_init(TC_TIME2_CH0, TIMER_MS);
    timer_start(TC_TIME2_CH0);

    printf("\r\n[CM7_0] nav ready. camera:%d gps:%d wifi:%d socket:%d dst:%s:%s local:%s\r\n",
           camera_init_ok,
           gps_init_ok,
           wifi_init_ok,
           socket_init_ok,
           WIFI_CAMERA_TARGET_IP,
           WIFI_CAMERA_TARGET_PORT,
           WIFI_CAMERA_LOCAL_PORT);

    while(true)
    {
        uint32 now = timer_get(TC_TIME2_CH0);
        frame_updated = 0;

        gps_updated = gps_service_update();
        if(gps_updated)
        {
            nav_bridge_notify_gps_update(now);
        }

        if(camera_init_ok && mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            camera_process_frame();
            update_camera_overlay();
            nav_bridge_notify_camera_frame(now);
            frame_updated = 1;
        }

        nav_bridge_update(now);
        bridge_debug = nav_bridge_get_debug();

        if(socket_init_ok && camera_init_ok && frame_updated)
        {
            seekfree_assistant_camera_send();
            assistant_send_telemetry();
        }

        if(now - last_log_ms >= CAMERA_STREAM_LOG_INTERVAL_MS)
        {
            last_log_ms = now;
            gpio_toggle_level(LED1);
            print_obstacle_validation_log(bridge_debug);
        }
    }
}
