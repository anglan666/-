#include "zf_common_headfile.h"

#include "pid.h"
#include "steer_control.h"

steer_control_struct steer_1;
steer_control_struct steer_2;
steer_control_struct steer_3;
steer_control_struct steer_4;

void steer_control_init(void)
{
    steer_1.pwm_pin             = STEER_1_PWM;
    steer_1.control_frequency   = STEER_1_FRE;
    steer_1.steer_dir           = STEER_1_DIR;
    steer_1.center_num          = STEER_1_CENTER;
    steer_1.now_angle           = STEER_1_ANGLE;
    steer_1.min_pwm             = STEER_1_MIN_PWM + MARGIN;//左上轮机最小pwm值加上500,留下余量,防止舵机过载
    steer_1.max_pwm             = STEER_1_MAX_PWM - MARGIN; //左上轮机最大pwm值减去500,留下余量,防止舵机过载   

    steer_2.pwm_pin             = STEER_2_PWM;
    steer_2.control_frequency   = STEER_2_FRE;
    steer_2.steer_dir           = STEER_2_DIR;
    steer_2.center_num          = STEER_2_CENTER;
    steer_2.now_angle           = STEER_2_ANGLE;
    steer_2.min_pwm             = STEER_2_MIN_PWM + MARGIN;//右上轮机最小pwm值加上500,留下余量,防止舵机过载
    steer_2.max_pwm             = STEER_2_MAX_PWM - MARGIN; //右上轮机最大pwm值减去500,留下余量,防止舵机过载   

    steer_3.pwm_pin             = STEER_3_PWM;
    steer_3.control_frequency   = STEER_3_FRE;
    steer_3.steer_dir           = STEER_3_DIR;
    steer_3.center_num          = STEER_3_CENTER;
    steer_3.now_angle           = STEER_3_ANGLE;
    steer_3.min_pwm             = STEER_3_MIN_PWM + MARGIN;//右下轮机最小pwm值加上500,留下余量,防止舵机过载
    steer_3.max_pwm             = STEER_3_MAX_PWM - MARGIN; //右下轮机最大pwm值减去500,留下余量,防止舵机过载   

    steer_4.pwm_pin             = STEER_4_PWM;
    steer_4.control_frequency   = STEER_4_FRE;
    steer_4.steer_dir           = STEER_4_DIR;
    steer_4.center_num          = STEER_4_CENTER;
    steer_4.now_angle           = STEER_4_ANGLE;
    steer_4.min_pwm             = STEER_4_MIN_PWM + MARGIN;//左下轮机最小pwm值加上500,留下余量,防止舵机过载
    steer_4.max_pwm             = STEER_4_MAX_PWM - MARGIN; //左下轮机最大pwm值减去500,留下余量,防止舵机过载   
    
    steer_1.center_num = steer_1.center_num - 300 * steer_1.steer_dir;
    steer_2.center_num = steer_2.center_num - 300 * steer_2.steer_dir;
    steer_3.center_num = steer_3.center_num - 300 * steer_3.steer_dir;
    steer_4.center_num = steer_4.center_num - 300 * steer_4.steer_dir;

    steer_1.now_location = steer_1.center_num ;
    steer_2.now_location = steer_2.center_num ;
    steer_3.now_location = steer_3.center_num ;
    steer_4.now_location = steer_4.center_num ;

    //
    pwm_init(steer_1.pwm_pin, steer_1.control_frequency, steer_1.now_location);
    pwm_init(steer_2.pwm_pin, steer_2.control_frequency, steer_2.now_location);
    pwm_init(steer_3.pwm_pin, steer_3.control_frequency, steer_3.now_location);
    pwm_init(steer_4.pwm_pin, steer_4.control_frequency, steer_4.now_location);
    
    //PID舵机控制初始化
 //pid初始化               Kp       Ki         Kd       I_value  积分限幅  输出限幅  pid类型
 PID_Init(&Steer_Roll,    20.0   ,   0.1,      1.0,      0.15,      800     ,  1500,    Position_PID);
    

}

void steer_duty_set(steer_control_struct *control_data, int16 duty)
{
    control_data->now_location =(int16)func_limit_ab(duty, control_data->min_pwm, control_data->max_pwm);
    pwm_set_duty(control_data->pwm_pin, control_data->now_location);
}

void steer_control (steer_control_struct *control_data, int16 delta_move_num)//delta_move_numΪ���ռ�ձȵı仯��
{
    control_data->now_location = control_data->now_location + delta_move_num * control_data->steer_dir;
    
    control_data->now_location = func_limit_ab(control_data->now_location, control_data->min_pwm, control_data->max_pwm);
    pwm_set_duty(control_data->pwm_pin, control_data->now_location);
}


