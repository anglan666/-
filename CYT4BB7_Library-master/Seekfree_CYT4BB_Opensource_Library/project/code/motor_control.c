
#include "zf_common_headfile.h"

#include "IMU.h"
#include "small_driver_uart_control.h"
#include "motor_control.h"
#include "pid.h"

MOTOR_DUTY motor_duty;
PID_Controller Angular_speed_cycle;//角速度环
PID_Controller Angle_cycle;//角度环
PID_Controller Speed_cycle;//速度环
PID_Controller Turn_yaw;//偏航角PID控制转弯

 float car_speed;             //实际车速
 float tar_speed         =0.0f;  //目标速度
 float yaw_angle_k       =20.0f;//偏航角补偿系数
 int turn_flag           =0;  //是否转弯(0：否，1：是)
 float turn_angle        =0;  //目标偏航角
void motor_init()                       
{
  small_driver_uart_init();
  motor_duty.left_duty=0;
  motor_duty.right_duty=0;
}


void motor_duty_set(int left_duty,int right_duty)
{
  left_duty=func_limit_ab(left_duty,-8000,8000);       //�޷���ռ�ձ�
  right_duty=func_limit_ab(right_duty,-8000,8000);    //�޷���ռ�ձ�
  if((func_abs(left_duty)>3000)||(func_abs(right_duty)>3000))
  {
    left_duty=0;
    right_duty=0;
  }
  
  small_driver_set_duty(left_duty,right_duty);
  
  return;
}
void balance_Init(void)
{
  motor_init();
  //pid��ʼ��                        Kp         Ki         Kd     I_value 积分限幅 输出限幅  PID类型
  // [调参归零] 从零开始，逐步增加P、I参数
  
  PID_Init(&Angular_speed_cycle,      1.0f,     0.1f,       0.1f,    0.1f,  1000.0f, 5000.0f,Position_PID);

  PID_Init(&Angle_cycle,             62.0f,     0.1f,       0.0f,   0.05f,  1000.0f, 5000.0f,Position_PID);

  PID_Init(&Speed_cycle,             7.0f,       0.5f,      0.0f,    0.5f,  1000.0f, 5000.0f,Position_PID);

  PID_Init(&Turn_yaw,                1.0f,       0.05f,      0.1f,    0.1f,  2000.0f, 5000.0f,Position_PID2);
}

void car_fact_speed_calculate()
{
  car_speed=(motor_value.receive_right_speed_data-motor_value.receive_left_speed_data ) / 2.0;
}

void balance_control(void)
{
    //1毫秒计算一次角速度环                                             
    Pid_Control(&Angular_speed_cycle,Angle_cycle.Out,imu963ra_gyro_y);
  
  if(imu_t%5==0)              //5毫秒计算一次角度环
    {    
      
        Pid_Control(&Angle_cycle,MACHINE_ZERO,eulerAngle.pitch);
     
    }
if(imu_t%20==0)              //20毫秒计算一次速度环
  {
      car_fact_speed_calculate();//计算实际车速
      Pid_Control(&Speed_cycle,tar_speed,car_speed);


}

  Pid_Control(&Turn_yaw,turn_angle,eulerAngle.yaw);

  //if(imu_t==5000) turn_angle=180;
  

   motor_duty.left_duty = (int)func_limit_ab((Angular_speed_cycle.Out+(1-turn_flag)*yaw_angle_k*eulerAngle.yaw-turn_flag*Turn_yaw.Out) , -3000, 3000);
   motor_duty .right_duty= (int)func_limit_ab((Angular_speed_cycle.Out-(1-turn_flag)*yaw_angle_k*eulerAngle.yaw+turn_flag*Turn_yaw.Out) , -3000, 3000);   
   //printf("%d,%d\r\n",motor_duty.left_duty,motor_duty .right_duty);
// printf("%f,%d,%d\n",eulerAngle.pitch,motor_duty.left_duty,motor_duty .right_duty);
    motor_duty_set((int16)-motor_duty.right_duty,(int16)-motor_duty.left_duty);
   

     return;
}
  