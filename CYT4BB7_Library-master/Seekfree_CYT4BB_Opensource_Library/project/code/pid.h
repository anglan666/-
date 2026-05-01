#ifndef PID_H_
#define PID_H_
typedef enum {
    Position_PID = 0,
    Position_PID2 = 2,
    Increment_PID = 1,
} PID_type;

typedef struct {

    PID_type Type;
    // 内部变量
    float Pre_Now;                   // 之前的当前值
    float Pre_Target;                // 之前的目标值
    float Pre_Out;                   // 之前的输出值
    float Pre_Error;                 // 前向误差

    // 输出值
    float Out;

    // PID系数
    float K_P;                       // PID的P
    float K_I;                       // PID的I
    float K_D;                       // PID的D
    float Integral_value ;           //积分程度系数 
   

    // 限幅值
    float I_Out_Max;                 // 积分限幅
    float Out_Max;                   // 输出限幅

    // 当前值和目标值
    float Target;                    // 目标值
    float Now;                       // 当前值

    //误差值
    float error;
    float error_last;
    float error_last_last;

    // 积分值
    float Integral_Error;            // 积分值

} PID_Controller;


void PID_Init(PID_Controller* pid, float K_P, float K_I, float K_D, float Integral_value  , float I_Out_Max, float Out_Max,PID_type Type);
void PID_Set_Target(PID_Controller* pid, float Target);

void PID_Set_Now(PID_Controller* pid, float Now);
void PID_TIM_Adjust_PeriodElapsedCallback(PID_Controller* pid) ;

void Pid_Control(PID_Controller *pid, float Target, float Now);

extern PID_Controller Angular_speed_cycle;//角速度环
extern PID_Controller Angle_cycle;//加速度环(角度环）
extern PID_Controller Speed_cycle;//速度环
extern PID_Controller Steer_Roll;
extern PID_Controller Turn_yaw;
#endif