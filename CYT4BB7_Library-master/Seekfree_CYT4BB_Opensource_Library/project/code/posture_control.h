
#ifndef _POSTURE_CONTROL_H
#define _POSTURE_CONTROL_H

#include "zf_common_headfile.h"

#define  ROLL_ZERO     (-0.0f)

void dynamic_steer_control(void);
void jump_control(void);
void bridge_control(void);
extern int16 jump_flag;
extern int16 run_flag;
extern int16 jumped_flag, jump_pre_flag;
extern int16 bridge_flag;
extern int16 bridge_slowdown_flag, bridge_step;
extern  float steer_balance_angle ;
extern int16 steer_location_offset[4];
extern int16 steer_target_offset[4];
#endif /* _POSTURE_CONTROL_H_ */
