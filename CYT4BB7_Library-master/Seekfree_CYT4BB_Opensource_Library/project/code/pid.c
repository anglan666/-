#include "zf_common_headfile.h"
#include "pid.h"



void PID_Init(PID_Controller* pid, float K_P, float K_I, float K_D, float Integral_value  , float I_Out_Max, float Out_Max,PID_type Type)
{
     pid->K_P = K_P;
     pid->K_I = K_I;
     pid->K_D = K_D;
     pid->Integral_value=Integral_value;        //���̶ֳ�ϵ��
     pid->I_Out_Max = I_Out_Max;               //�����޷�
     pid->Out_Max = Out_Max;                  //����޷�

     pid->Pre_Now = 0.0f;
     pid->Pre_Target = 0.0f;
     pid->Pre_Out = 0.0f;
     pid->Pre_Error = 0.0f;

     pid->Out = 0.0f;
     pid->Target = 0.0f;
     pid->Now = 0.0f;
     pid->Integral_Error = 0.0f;
     
     
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->error_last_last = 0.0f;


     pid->Type = Type;
}
void PID_Set_Target(PID_Controller* pid, float Target)
{
    pid->Target = Target;
    //    printf("pid->Target=%.2f\n",pid->Target);
}

void PID_Set_Now(PID_Controller* pid, float Now)
{
    //    float now=(float)Now;
    pid->Now = Now;
    // printf("now:%.2f, pid->Now:%.2f\r\n", now, pid->Now);
}

void PID_TIM_Adjust_PeriodElapsedCallback(PID_Controller* pid) 
{
    // ʵ�� PID �����߼�
    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;

    switch (pid->Type)
{
     case Position_PID://λ��ʽ
     {
         pid->error = pid->Target - pid->Now;
         //pid->error = pid->Target - pid->Now;
         // ���� p ��
         p_out = pid->K_P * pid->error;
         //���������
         pid->Integral_Error += (pid->error*pid->Integral_value);
         pid->Integral_Error = func_limit_ab(pid->Integral_Error, -pid->I_Out_Max,pid->I_Out_Max);//�����޷�

         i_out = pid->K_I * pid->Integral_Error;
         //����΢����
         d_out = (pid->error - pid->Pre_Error) * pid->K_D;

         pid->Out = p_out + i_out + d_out;
         //������޷�
         pid->Out = func_limit_ab(pid->Out, -pid->Out_Max,pid->Out_Max);
         //    printf("pid->Out:%.2f,p_out:%.2f\r\n", pid->Out,p_out);
             // ������ʷֵ
         pid->Pre_Now = pid->Now;
         pid->Pre_Target = pid->Target;
         pid->Pre_Out = pid->Out;
         pid->Pre_Error = pid->error;
         break;
     }
     case Position_PID2://λ��ʽ
     {
         pid->error =  0.8 * (pid->Target - pid->Now) + pid->error_last * 0.2;

         // ���� p ��
         p_out = pid->K_P * pid->error;

         pid->Integral_Error += pid->error;

         pid->Integral_Error = func_limit_ab(pid->Integral_Error, -pid->I_Out_Max,pid->I_Out_Max);//�����޷�

         i_out = pid->K_I * pid->Integral_Error;

         d_out = (pid->error - pid->Pre_Error) * pid->K_D;

         pid->Out = p_out + i_out + d_out;
         //����޷�
         pid->Out = func_limit_ab(pid->Out, -pid->Out_Max,pid->Out_Max);
         //printf("pid->Out:%.2f,p_out:%.2f\r\n", pid->Out,p_out);
          // ������ʷֵ
         pid->Pre_Now = pid->Now;
         pid->Pre_Target = pid->Target;
         pid->Pre_Out = pid->Out;
         pid->Pre_Error = pid->error;
    
         break;
     }
     case Increment_PID://����ʽ
     {
         pid->error = pid->Target - pid->Now;

         pid->Out += pid->K_P * (pid->error - pid->error_last) + pid->K_I * pid->error + pid->K_D * (pid->error - 2 * pid->error_last + pid->error_last_last);

         pid->Out = func_limit(pid->Out, pid->Out_Max);
         
         pid->error_last_last = pid->error_last;
         pid->error_last = pid->error;
         break;
     }
 
 }

}
void Pid_Control(PID_Controller *pid, float Target, float Now)
{
    PID_Set_Target(pid, Target);
    PID_Set_Now(pid, Now);
    PID_TIM_Adjust_PeriodElapsedCallback(pid);
}