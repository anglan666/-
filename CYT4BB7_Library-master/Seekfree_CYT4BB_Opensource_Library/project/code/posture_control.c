
/*
 *  posture_control.c
 *
 *  Created on: 2025��3��9��
 *      Author: NYC
 */

#include "posture_control.h"
#include "zf_common_headfile.h"
#include "IMU.h"
#include "motor_control.h"
#include "pid.h"
#include "steer_control.h"

int time=0;

int16 run_flag          = 1;
int16 jump_flag         = 2;
int16 bridge_flag       = 0;
int16 mine_rotate_flag  = 0;

int16 jump_time_counter = 0;          //跳跃时间计数器
float error_hight       = 0;          //舵机高度差
float steer_k           = 0.0f;       //俯仰角补偿系数
int16 steer_output_duty = 0;
int   B_X               = 0,
      B_H               = 0;
PID_Controller Steer_Roll;//舵机横滚角PID控制
    int16 steer_location_offset[4] = {0};
    int16 steer_target_offset[4] = {0};
 float steer_balance_angle = 0;
static float Steer_Roll_Last = 0;
static float steer_output_duty_filter = 0;
static int bridge_duty_limit = 0;
static int jump_distance=65;
static float default_tar_speed = 200.0f;
static float bridge_tar_speed = 80.0f;
static float default_yaw_angle_k = 20.0f;
static float bridge_yaw_angle_k = 30.0f;
void dynamic_steer_control(void)
{
   if(turn_flag == 0)
   {
       tar_speed = default_tar_speed;
   }
   yaw_angle_k = default_yaw_angle_k;

   if(mine_rotate_flag)
   {
       tar_speed = 0.0f;
       yaw_angle_k = 0.0f;
   }

   /* 纯视觉起跳触发已接管，这里不再使用 TOF 自动触发 jump_flag */
   

    // Calculate steering output duty based on speed
    steer_output_duty = -(int16)func_limit_ab( -Speed_cycle.Out/7 , -350, 350) * 6;
   // 
    // 
    steer_output_duty_filter = (steer_output_duty_filter * 15 + (float)steer_output_duty*5) / 20.0f;
   // printf("%d,%f,%d,%f\r\n",imu_t,Speed_cycle.Out,steer_output_duty,steer_output_duty_filter);
    steer_balance_angle = 0;
    
  if(bridge_flag==1)
  {
    if(turn_flag == 0)
    {
      tar_speed = bridge_tar_speed;
    }
    yaw_angle_k = bridge_yaw_angle_k;  //增强偏航角的补偿作用，避免过桥后方向不对
    steer_output_duty_filter=0.7*steer_output_duty_filter;
    Pid_Control(&Steer_Roll,ROLL_ZERO,eulerAngle.roll);
    //steer_balance_angle = func_limit_ab(Steer_Roll.Out * 0.8 + Steer_Roll_Last * 0.2, -200, 200);
    steer_balance_angle=Steer_Roll.Out * 0.9 + Steer_Roll_Last * 0.1;
    Steer_Roll_Last = Steer_Roll.Out * 0.9 + Steer_Roll_Last * 0.1;
    //printf("%f,%f,%f\n",steer_balance_angle,Steer_Roll.Out,eulerAngle.roll);
  }

//--------车模布局（车头）--------
//---steer1-------------steer2----
//---steer4-------------steer3----
    
    // Calculate steering location offsets for each wheel

    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;


    // Calculate target offsets for each wheel
    // 左侧舵机：1和4，当左倾时(steer_balance_angle>0)需要升高
    // 右侧舵机：2和3，当右倾时(steer_balance_angle<0)需要升高
    // 注：steer_control中会乘以steer_dir，这里直接按绝对值补偿
    steer_target_offset[1] =(int)(  steer_output_duty_filter +  (steer_balance_angle > 0 ? func_abs(steer_balance_angle) : 0.5* func_abs(steer_balance_angle)));    // 右侧舵机2
    steer_target_offset[0] =(int)(  steer_output_duty_filter +  (steer_balance_angle < 0 ? func_abs(steer_balance_angle) : 0.5* func_abs(steer_balance_angle)));     // 左侧舵机1

    steer_target_offset[2] =(int)( - steer_output_duty_filter +  (steer_balance_angle > 0 ? func_abs(steer_balance_angle) : 0.5* func_abs(steer_balance_angle))); // 右侧舵机3
    steer_target_offset[3] =(int)( - steer_output_duty_filter +  (steer_balance_angle < 0 ? func_abs(steer_balance_angle) : 0.5* func_abs(steer_balance_angle)));  // 左侧舵机4

    
    if (run_flag == 1)
    {
        if(mine_rotate_flag)
        {
            steer_control(&steer_1, func_limit_ab(20 - steer_location_offset[0], -40, 40));
            steer_control(&steer_2, func_limit_ab(20 - steer_location_offset[1], -40, 40));
            steer_control(&steer_3, func_limit_ab(-20 - steer_location_offset[2], -40, 40));
            steer_control(&steer_4, func_limit_ab(-20 - steer_location_offset[3], -40, 40));
            return;
        }

         if(jump_flag == 1)
        {
          steer_balance_angle=func_limit_ab(Angle_cycle.Out/7,-300,300)*6;
           jump_control();
           return;
        }
        if(bridge_flag == 1)
        {
            float error_abs = func_abs(eulerAngle.roll - ROLL_ZERO);
             if(error_abs > 15.0f)
               bridge_duty_limit = 150;   // 大倾角时快速响应
              else if(error_abs > 5.0f)
                bridge_duty_limit = 90;   // 中等倾角
              else
                bridge_duty_limit = 40;    // 接近平衡时精细调整
           steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -bridge_duty_limit, bridge_duty_limit));
           steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -bridge_duty_limit, bridge_duty_limit));
           steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -bridge_duty_limit, bridge_duty_limit));
           steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -bridge_duty_limit, bridge_duty_limit));
          return;
        }
         
        
            // Control steering based on target and location offsets
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -10, 10));
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -10, 10));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -10, 10));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -10, 10));
        
        
    }

}
void bridge_control(void)
{
    Pid_Control(&Steer_Roll,0,eulerAngle.roll);
    steer_balance_angle=0;
    steer_output_duty_filter = (int16)func_limit_ab( Steer_Roll.Out/7 , -40, 40) * 6;
   
     steer_output_duty_filter = (steer_output_duty_filter * 10 + (float)steer_output_duty*10) / 20.0f;
     //printf("%f,%f,%f\n",steer_output_duty_filter,Steer_Roll.Out,eulerAngle.roll);
    
    steer_duty_set(&steer_1, (int)(steer_1.now_location +steer_1.steer_dir* steer_output_duty_filter));
    steer_duty_set(&steer_2, (int)(steer_2.now_location -steer_2.steer_dir* steer_output_duty_filter));
    steer_duty_set(&steer_3, (int)(steer_3.now_location -steer_3.steer_dir* steer_output_duty_filter));
    steer_duty_set(&steer_4, (int)(steer_4.now_location +steer_4.steer_dir* steer_output_duty_filter));
  
    /*
    error_hight   = (LETF_TO_RIGHT) * sin(Steer_Roll.Out * (PI/180));
     X_Y_Set(B_X,B_H+error_hight/2.0f,B_H-error_hight/2.0f);
    inverseKinematics();
*/

}


int16 jump_ready_time           =20,    //准备时间(下蹲)
      jump_go_time              =140,    //起跳时间
      jump_back_time            =60,//收脚时间
      jump_buffer_ready_time    =55,//缓冲准备时间
      jump_buffer_time          =60;  //缓冲时间
//int16 steer_last_location[4] = {0};

void jump_control(void)
{
    jump_time_counter++;
    if(jump_time_counter<jump_ready_time)//准备阶段
    {
        steer_duty_set(&steer_1,steer_1.center_num -steer_1.steer_dir* 300);
        steer_duty_set(&steer_2,steer_2.center_num -steer_2.steer_dir* 300);
        steer_duty_set(&steer_3,steer_3.center_num -steer_3.steer_dir* 300);
        steer_duty_set(&steer_4,steer_4.center_num -steer_4.steer_dir* 300);
    }
   else if(jump_time_counter<jump_ready_time+jump_go_time)//起跳阶段
   {
    /*steer_last_location[0]=steer_1.now_location;
    steer_last_location[1]=steer_2.now_location;
    steer_last_location[2]=steer_3.now_location;
    steer_last_location[3]=steer_4.now_location; */   
    steer_duty_set(&steer_1,steer_1.center_num +steer_1.steer_dir* 3000);
    steer_duty_set(&steer_2,steer_2.center_num +steer_2.steer_dir* 3000);
    steer_duty_set(&steer_3,steer_3.center_num +steer_3.steer_dir* 3000);
    steer_duty_set(&steer_4,steer_4.center_num +steer_4.steer_dir* 3000);
   }
   else if(jump_time_counter<jump_ready_time+jump_go_time+jump_back_time)//收脚阶段
   {
    steer_duty_set(&steer_1,steer_1.center_num -steer_1.steer_dir* 800);
    steer_duty_set(&steer_2,steer_2.center_num -steer_2.steer_dir* 800);
    steer_duty_set(&steer_3,steer_3.center_num -steer_3.steer_dir* 800);
    steer_duty_set(&steer_4,steer_4.center_num -steer_4.steer_dir* 800);
   }
   else if(jump_time_counter<jump_ready_time+jump_go_time+jump_back_time+jump_buffer_ready_time)//缓冲准备阶段
   {
    steer_duty_set(&steer_1,steer_1.center_num +steer_1.steer_dir* 300);
    steer_duty_set(&steer_2,steer_2.center_num +steer_2.steer_dir* 300);    
    steer_duty_set(&steer_3,steer_3.center_num +steer_3.steer_dir* 300);
    steer_duty_set(&steer_4,steer_4.center_num +steer_4.steer_dir* 300);

   }
   else if(jump_time_counter<jump_ready_time+jump_go_time+jump_back_time+jump_buffer_ready_time+jump_buffer_time)//缓冲执行阶段
   {
    steer_control(&steer_1,14);
    steer_control(&steer_2,14);
    steer_control(&steer_3,14);
    steer_control(&steer_4,14);
   }
   else
   {
    jump_flag = 0;
    jump_time_counter = 0;
   }
}



