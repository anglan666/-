#ifndef STEER_CONTROL_H_
#define STEER_CONTROL_H_

#include "zf_common_headfile.h"

 //--------车模布局（车头）--------
//---steer1-------------steer2----
//---steer4-------------steer3----

#define STEER_1_PWM         (TCPWM_CH25_P09_1)   // 左上轮机控制引脚   
#define STEER_1_FRE         (300)               // 左上轮机控制频率
#define STEER_1_DIR         (1)                // 左上轮机旋转方向（pwm值越大，舵机往下转）
#define STEER_1_CENTER      (4350)              // 左上轮机中心值（初始保持位置）
#define STEER_1_ANGLE      (90)              // 左上轮机角度（初始保持位置）
#define STEER_1_MIN_PWM     (1400)              // 左上轮机最小pwm值
#define STEER_1_MAX_PWM     (7300)              // 左上轮机最大pwm值

#define STEER_2_PWM         (TCPWM_CH31_P10_3)   // 右上轮机控制引脚   
#define STEER_2_FRE         (300)               // 右上轮机控制频率
#define STEER_2_DIR         (-1)                 // 右上轮机旋转方向（pwm值越大，舵机往上转）
#define STEER_2_CENTER      (4500)              // 右上轮机中心值（初始保持位置）4600
#define STEER_2_ANGLE      (90)              // 右上轮机角度（初始保持位置）
#define STEER_2_MIN_PWM     (1600)              // 右上轮机最小pwm值
#define STEER_2_MAX_PWM     (7400)              // 右上轮机最大pwm值

#define STEER_3_PWM         (TCPWM_CH24_P09_0)   // 右下轮机控制引脚     
#define STEER_3_FRE         (300)               // 右下轮机控制频率
#define STEER_3_DIR         (1)                 // 右下轮机旋转方向（pwm值越大，舵机往下转）
#define STEER_3_CENTER      (4000)              // 右下轮机中心值（初始保持位置）  4100
#define STEER_3_ANGLE      (90)              // 右下轮机角度（初始保持位置）
#define STEER_3_MIN_PWM     (1100)              // 右下轮机最小pwm值
#define STEER_3_MAX_PWM     (6900)              // 右下轮机最大pwm值

#define STEER_4_PWM         (TCPWM_CH30_P10_2)   // 左下轮机控制引脚   
#define STEER_4_FRE         (300)               // 左下轮机控制频率
#define STEER_4_DIR         (-1)                // 左下轮机旋转方向（pwm值越大，舵机往上转）
#define STEER_4_CENTER      (4650)              // 左下轮机中心值（初始保持位置）  
#define STEER_4_ANGLE      (90)              // 左下轮机角度（初始保持位置）
#define STEER_4_MIN_PWM     (1800)              // 左下轮机最小pwm值
#define STEER_4_MAX_PWM     (7500)              // 左下轮机最大pwm值

#define MARGIN              (1000)             //余量

typedef struct
{
    pwm_channel_enum pwm_pin;   // �����������
    int16 control_frequency;    // �������Ƶ��
    int16 steer_dir;            // �����ת����
    int16 center_num;           // �������ֵ����ʼ����λ�ã�
    int16 now_location;         // �����ǰ���λ��
    int16 now_angle;            // �����ǰ���角度
    int16 min_pwm;              // ������Сֵ
    int16 max_pwm;              // �������ֵ
} steer_control_struct;

// �ʼǣ�����λ��Ϊ�ĸ��ȱ�ƽ���ڵ��棬�߶�Ϊ2400����߸߶�Ϊ5400���ﵽ��߸߶�ʱ���ĸ��ȱ۴�ֱ�ڵ���

extern steer_control_struct steer_1;
extern steer_control_struct steer_2;
extern steer_control_struct steer_3;
extern steer_control_struct steer_4;

void steer_control_init(void);
void steer_duty_set(steer_control_struct *control_data, int16 duty);
void steer_control(steer_control_struct *control_data, int16 move_num);












#endif