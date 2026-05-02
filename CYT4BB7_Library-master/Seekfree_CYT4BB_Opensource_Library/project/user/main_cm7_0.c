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
#define KEY1                          (P20_3)
#define IPC_SEND_INTERVAL_MS          (20u)
#define GPS_POLL_INTERVAL_MS          (10u)
#define LOG_PRINT_INTERVAL_MS         (200u)
#define ASSISTANT_SEND_EVERY_N_FRAMES (1u)

static void ipc_dummy_callback(uint32 ipc_data)
{
    (void)ipc_data;
}

void gps_debug_callback(void)
{
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

int main(void)
{
    uint8 camera_init_ok = 0;
    uint8 send_enable = 0;
    uint8 key_last = 1;
    uint32 frame_count = 0;
    uint32 last_gps_poll_ms = 0;
    uint32 last_nav_send_ms = 0;
    uint32 last_log_ms = 0;
    uint32 last_camera_frame_ms = 0;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        mt9v03x_image[0],
        MT9V03X_W,
        MT9V03X_H
    );

    camera_init_ok = (camera_service_init() == 0);
    gps_service_init();
    ipc_communicate_init(IPC_PORT_1, ipc_dummy_callback);
    nav_bridge_init();

    timer_init(TC_TIME2_CH0, TIMER_MS);
    timer_start(TC_TIME2_CH0);

    printf("\r\n[CM7_0] upper core ready. camera:%d\r\n", camera_init_ok);

    while(true)
    {
        uint32 now = timer_get(TC_TIME2_CH0);
        uint8 key = gpio_get_level(KEY1);

        if(key_last && !key)
        {
            send_enable = !send_enable;
            printf("[CM7_0] image send %s\r\n", send_enable ? "on" : "off");
        }
        key_last = key;

        if(now - last_gps_poll_ms >= GPS_POLL_INTERVAL_MS)
        {
            last_gps_poll_ms = now;
            if(gps_service_update())
            {
                nav_bridge_notify_gps_update(now);
            }
        }

        if(camera_init_ok && mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            frame_count++;
            last_camera_frame_ms = now;

            camera_process_frame();
            nav_bridge_notify_camera_frame(now);

            if(send_enable && (frame_count % ASSISTANT_SEND_EVERY_N_FRAMES == 0))
            {
                seekfree_assistant_camera_send();
            }
        }

        if(now - last_nav_send_ms >= IPC_SEND_INTERVAL_MS)
        {
            last_nav_send_ms = now;
            nav_bridge_update(now);
        }

        if(now - last_log_ms >= LOG_PRINT_INTERVAL_MS)
        {
            const nav_bridge_debug_struct *bridge_debug = nav_bridge_get_debug();

            last_log_ms = now;
            gpio_toggle_level(LED1);
            printf("[DEC] rv:%d road:%s task:%s src:%s steer:%.2f spd:%.2f | ipc_spd:%d ipc_turn:%d flags:0x%02X\r\n",
                   track_info.is_valid,
                   road_type_name(track_info.road_type),
                   nav_task_name(nav_output.current_task),
                   nav_source_name(nav_output.active_source),
                   nav_output.steer_angle,
                   nav_output.target_speed,
                   bridge_debug->speed_cmd,
                   bridge_debug->turn_cmd,
                   bridge_debug->flags);
        }
    }
}
