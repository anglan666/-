/*********************************************************************************************************************
* CM7_1 底盘控制核心
* - 负责 IMU / TOF / 电机 / 舵机初始化
* - 负责 1ms 高频稳定控制环
* - 通过 IPC 接收来自 CM7_0 的轻量控制命令
*********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "../code/IMU.h"
#include "../code/motor_control.h"
#include "../code/posture_control.h"
#include "../code/TOF.h"
#include "../code/steer_control.h"
#include "../code/navigation_bridge.h"

#define CHASSIS_LOG_INTERVAL_MS    (200u)

int imu_t = 0;
volatile uint8 g_chassis_ipc_new_data = 0;
volatile int16 g_chassis_cmd_speed = 0;
volatile int16 g_chassis_cmd_turn_delta = 0;
volatile uint8 g_chassis_cmd_flags = 0;

static void chassis_ipc_callback(uint32 ipc_data)
{
    uint8 cmd = (uint8)((ipc_data >> 24) & 0xFFu);
    uint16 raw = (uint16)(ipc_data & 0xFFFFu);

    switch(cmd)
    {
        case CHASSIS_IPC_CMD_SPEED:
            g_chassis_cmd_speed = (int16)raw;
            g_chassis_ipc_new_data = 1;
            break;

        case CHASSIS_IPC_CMD_TURN:
            g_chassis_cmd_turn_delta = (int16)raw;
            g_chassis_ipc_new_data = 1;
            break;

        case CHASSIS_IPC_CMD_FLAGS:
            g_chassis_cmd_flags = (uint8)(raw & 0x00FFu);
            g_chassis_ipc_new_data = 1;
            break;

        default:
            break;
    }
}

static void chassis_apply_ipc_targets(void)
{
    float turn_delta;

    if(!g_chassis_ipc_new_data)
    {
        return;
    }

    tar_speed = (float)g_chassis_cmd_speed;

    turn_delta = (float)g_chassis_cmd_turn_delta / NAV_BRIDGE_STEER_PACKET_SCALE;
    turn_angle = (float)eulerAngle.yaw + turn_delta;
    turn_flag = (g_chassis_cmd_flags & CHASSIS_IPC_FLAG_TURN_EN) ? 1 : 0;

    bridge_flag = (g_chassis_cmd_flags & CHASSIS_IPC_FLAG_BRIDGE_EN) ? 1 : 0;
    mine_rotate_flag = (g_chassis_cmd_flags & CHASSIS_IPC_FLAG_MINE_ROT_EN) ? 1 : 0;

    if(g_chassis_cmd_flags & CHASSIS_IPC_FLAG_JUMP_EN)
    {
        if(jump_flag != 1)
        {
            jump_flag = 1;
        }
    }
    else if(jump_flag == 0)
    {
        jump_flag = 2;
    }

    g_chassis_ipc_new_data = 0;
}

int main(void)
{
    uint32 last_log_ms = 0;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    IMU_init();
    steer_control_init();
    balance_Init();
    tof_init();

    ipc_communicate_init(IPC_PORT_2, chassis_ipc_callback);

    timer_init(TC_TIME2_CH0, TIMER_MS);
    timer_start(TC_TIME2_CH0);

    pit_ms_init(PIT_CH0, 1);

    printf("\r\n[CM7_1] chassis core ready.\r\n");

    while(true)
    {
        uint32 now = timer_get(TC_TIME2_CH0);

        chassis_apply_ipc_targets();

        if(now - last_log_ms >= CHASSIS_LOG_INTERVAL_MS)
        {
            last_log_ms = now;
            printf("[CM7_1] car:%.2f tar:%.2f yaw:%.2f turn:%.2f | ipc_spd:%d ipc_turn:%d flags:0x%02X tf:%d jf:%d bf:%d mf:%d\r\n",
                   car_speed,
                   tar_speed,
                   eulerAngle.yaw,
                   turn_angle,
                   g_chassis_cmd_speed,
                   g_chassis_cmd_turn_delta,
                   g_chassis_cmd_flags,
                   turn_flag,
                   jump_flag,
                   bridge_flag,
                   mine_rotate_flag);
        }
    }
}
