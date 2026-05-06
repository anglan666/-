#ifndef NAVIGATION_BRIDGE_H_
#define NAVIGATION_BRIDGE_H_

/*
 * navigation_bridge.h
 *
 * CM7_0 侧桥接层职责：
 *   - 汇总 camera_service / gps_service / nav_decision 的输入输出
 *   - 调用 nav_update()
 *   - 将 nav_output 编码成 IPC 轻量控制命令发送给 CM7_1
 *
 * CM7_1 侧只消费编码后的目标量，不直接依赖上层决策模块。
 */

#include "zf_common_headfile.h"

#define NAV_BRIDGE_SPEED_SCALE        (200.0f)
#define NAV_BRIDGE_SPEED_MAX          (300.0f)
#define NAV_BRIDGE_SPEED_MIN          (-300.0f)
#define NAV_BRIDGE_STEER_SCALE        (1.0f)
#define NAV_BRIDGE_STEER_MAX          (25.0f)
#define NAV_BRIDGE_STEER_PACKET_SCALE (100.0f)

#define CHASSIS_IPC_CMD_SPEED         (0x01u)
#define CHASSIS_IPC_CMD_TURN          (0x02u)
#define CHASSIS_IPC_CMD_FLAGS         (0x03u)

#define CHASSIS_IPC_FLAG_TURN_EN      (1u << 0)
#define CHASSIS_IPC_FLAG_JUMP_EN      (1u << 1)
#define CHASSIS_IPC_FLAG_BRIDGE_EN    (1u << 2)
#define CHASSIS_IPC_FLAG_MINE_ROT_EN  (1u << 3)

typedef struct
{
    int16 speed_cmd;
    int16 turn_cmd;
    uint8 flags;
} nav_bridge_debug_struct;

void nav_bridge_init(void);
void nav_bridge_update(uint32 now_ms);
void nav_bridge_notify_camera_frame(uint32 now_ms);
void nav_bridge_notify_gps_update(uint32 now_ms);
const nav_bridge_debug_struct* nav_bridge_get_debug(void);

#endif /* NAVIGATION_BRIDGE_H_ */
