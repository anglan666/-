#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_


#define MACHINE_ZERO      (10.0f)

typedef struct{
  int left_duty;
  int right_duty;
}MOTOR_DUTY;    
extern int imu_t;   //ÖÐ¶Ï¼ÆÊýÆ÷
extern float car_speed,tar_speed;
extern MOTOR_DUTY motor_duty;
extern int turn_flag;
extern float turn_angle;
extern float yaw_angle_k;



void motor_init(void); 
void motor_duty_set(int left_duty,int right_duty) ;
void balance_Init(void);
void balance_control(void);






#endif