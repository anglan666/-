

#include "IMU.h"

#define LED1                          (P19_0)

//===================================================全局变量定义===================================================
/**
 * @defgroup IMU_Globals_Def 全局变量定义
 * @brief 模块内部使用的全局变量定义
 * @{
 */

imu_info_struct imu;                                    /*!< IMU数据结构体实例，包含所有传感器数据 */
eulerAngle_info_struct eulerAngle;                      /*!< 欧拉角结构体实例，存储解算后的姿态 */

float imu_kp = 0.4;                                    /*!< 互补滤波比例增益，与备份文件一致 */
float imu_ki = 0.015;                                 /*!< 互补滤波积分增益，与备份文件一致 */
float first_count_time = 0.0f;
/** @} */

//===================================================函数实现===================================================
/**
 * @defgroup IMU_Functions_Impl 函数实现
 * @brief 模块函数的详细实现
 * @{
 */

/**
 * @brief IMU初始化函数
 * @details 初始化IMU相关的结构体和变量，包括：
 *          - 初始化陀螺仪计数器和校准标志
 *          - 清零加速度和陀螺仪数据数组
 *          - 初始化欧拉角为0
 *          - 初始化偏移量为0
 *          - 初始化四元数误差积分为0
 * @note 应在系统启动时调用，确保传感器处于已知初始状态
 * @warning 此函数不会执行传感器硬件初始化，需单独调用传感器驱动初始化
 */
void IMU_init(void)
{//检测IMU传感器是否初始化成功
 while(1)
    {
#if IMU_TYPE
        if(imu963ra_init())
        {
          gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
           printf("\r\n imu963ra init error.");    
           gpio_set_level(LED1, GPIO_LOW);                                                             
        }
        else
        {
           break;
        }
#else
        if(imu660ra_init())
        {
          gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
           printf("\r\n imu660ra init error.");    
           gpio_set_level(LED1, GPIO_LOW);                                                             
        }
        else
        {
           break;
        }
#endif
    }
    
    // 初始化加速度计和陀螺仪数据数组
    for (int i = 0; i < 3; i++) {
        imu.acc.acc[i] = 0.0f;          // 加速度数据清空
        imu.acc.angle[i] = 0.0f;        // 加速度计算角度清零
        imu.acc.error[i] = 0.0f;          // 加速度计误差清空
        imu.acc.offset[i] = 0.0f;          // 加速度计偏移清空
        
        imu.gyro.gyro[i] = 0.0f;        // 陀螺仪数据清零
        imu.gyro.angle[i] = 0.0f;       // 陀螺仪积分角度清零
        imu.gyro.zero_angle[i] = 0.0f;  // 零点角度值清零
        imu.gyro.last_angle[i] = 0.0f;  // 上一次角度值清零
        imu.gyro.offset[i] = 0.0f;          // 陀螺仪偏移量清零
    }
    imu.acc.filtered[X] = (float)ACC_DATA_X/ACC_SPL;
    imu.acc.filtered[Y] = (float)ACC_DATA_Y/ACC_SPL;
    imu.acc.filtered[Z] = (float)ACC_DATA_Z/ACC_SPL;
    
    // 初始化欧拉角为0
    eulerAngle.pitch = 0.0f;   //抬头为正
    eulerAngle.roll = 0.0f;     //左倾为正
    eulerAngle.yaw = 0.0f;       //左转为正

    imu.quaternion.qua[0] = 1.0f;  
    imu.quaternion.qua[1] = 0.0f;  
    imu.quaternion.qua[2] = 0.0f;  
    imu.quaternion.qua[3] = 0.0f;  

    
    IMU_Zero_Bias_correction();
    
}

/**
 * @brief 获取传感器原始数据
 * @details 根据IMU_TYPE宏定义，从对应的IMU传感器读取数据
 *          - IMU_TYPE=1: 读取imu963ra数据
 *          - IMU_TYPE=0: 读取imu660ra数据
 * @note 建议在1ms定时中断中调用，保证数据更新频率为1kHz
 * @warning 此函数只读取数据到全局变量，不进行任何数据处理
 * @see IMU_TYPE
 */
void IMU_get_data(void)
{
#if IMU_TYPE
    imu963ra_get_acc();     /* 获取imu963ra加速度计原始数据 */
    imu963ra_get_gyro();    /* 获取imu963ra陀螺仪原始数据 */
    imu963ra_get_mag();
#else
    imu660ra_get_acc();     /*!< 获取imu660ra加速度计原始数据 */
    imu660ra_get_gyro();    /*!< 获取imu660ra陀螺仪原始数据 */
#endif
}
void IMU_Zero_Bias_correction(void)
{
    for (int i = 0; i < 200; i++) {
        IMU_get_data();
       #if IMU_TYPE
            imu.gyro.offset[X] += imu963ra_gyro_y;
            imu.gyro.offset[Y] += imu963ra_gyro_x;
            imu.gyro.offset[Z] += imu963ra_gyro_z;
        #else
            imu.gyro.offset[X] += imu660ra_gyro_x;
            imu.gyro.offset[Y] += imu660ra_gyro_y;
            imu.gyro.offset[Z] += imu660ra_gyro_z;
        #endif
        system_delay_ms(5);
    }
    // 保留重力方向参考：加速度不做静止均值直减
    imu.acc.offset[X] /= 200.0f;
    imu.acc.offset[Y] /= 200.0f;
    imu.acc.offset[Z] /= 200.0f;

    imu.gyro.offset[X] /= 200.0f;
    imu.gyro.offset[Y] /= 200.0f;
    imu.gyro.offset[Z] /= 200.0f;
}

void IMU_data_processing(void)
{
#if IMU_TYPE
    // imu963ra 陀螺仪零偏校准
    imu963ra_gyro_x -= imu.gyro.offset[Y];
    imu963ra_gyro_y -= imu.gyro.offset[X];
    imu963ra_gyro_z -= imu.gyro.offset[Z]*1.2f;

    // 噪声过滤：小于5的数据视为噪声，置为0
    if(func_abs(imu963ra_gyro_y) <= 5)
    {
        imu963ra_gyro_y = 0;
    }
    if(func_abs(imu963ra_gyro_x) <= 5)
    {
        imu963ra_gyro_x = 0;
    }
    if(func_abs(imu963ra_gyro_z) <= 5)
    {
        imu963ra_gyro_z = 0;
    }
#else
    // imu660ra 陀螺仪零偏校准
    imu660ra_gyro_x -= imu.gyro.offset[X];
    imu660ra_gyro_y -= imu.gyro.offset[Y];
    imu660ra_gyro_z -= imu.gyro.offset[Z];
    
    // 噪声过滤：小于5的数据视为噪声，置为0
    if(func_abs(imu660ra_gyro_x) <= 5)
    {
        imu660ra_gyro_x = 0;
    }
    if(func_abs(imu660ra_gyro_y) <= 5)
    {
        imu660ra_gyro_y = 0;
    }
    if(func_abs(imu660ra_gyro_z) <= 5)
    {
        imu660ra_gyro_z = 0;
    }
#endif
}
static void lowpass_filter(float *raw_x,float *raw_y,float *raw_z,float *filtered_x,float *filtered_y,float *filtered_z,float alpha)
{
  *filtered_x=alpha*(*filtered_x)+ (1-alpha)*(*raw_x);
  *filtered_y=alpha*(*filtered_y)+ (1-alpha)*(*raw_y);
  *filtered_z=alpha*(*filtered_z)+ (1-alpha)*(*raw_z);
}
static void normalize(float *ax,float *ay,float *az)
{
  float norm=sqrt(*ax*(*ax)+(*ay*(*ay)+(*az*(*az))));
  if(norm<0.1f)
  {
    *ax=0;
    *ay=0;
    *az=1.0f;
   return;
  }
  *ax/=norm;
  *ay/=norm;
  *az/=norm;
}
static bool is_static_state(float ax, float ay, float az)
{
    float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
    return (acc_magnitude > 0.9f && acc_magnitude < 1.1f);
}
void IMU_update_angle(void)
{
   float length;                        // 四元数模长（用于归一化）
    float x, y, z;                       // 陀螺仪角速度（弧度/秒）

// ===================== 1. 陀螺仪数据转换 =====================
// 原始数据 -> (°/s) -> 弧度/秒（先除以10再乘10做简单滤波）
// 0.01745329f 为度转弧度系数 (π/180)
    x = (float)(GYRO_DATA_X / 10 * 10) / GYRO_SPL * 0.01745329f;
    y = (float)(GYRO_DATA_Y / 10 * 10) / GYRO_SPL * 0.01745329f;
    z = (float)(GYRO_DATA_Z / 10 * 10) / GYRO_SPL * 0.01745329f;

// ===================== 2. 加速度计数据转换 =====================
// 原始数据 -> g 为单位 (1g ≈ 9.8 m/s²)
    float ax_g = (float)ACC_DATA_X / ACC_SPL;
    float ay_g = (float)ACC_DATA_Y / ACC_SPL;
    float az_g = (float)ACC_DATA_Z / ACC_SPL;

// ===================== 3. 静止状态判断与滤波系数自适应 =====================
    bool static_state = is_static_state(ax_g, ay_g, az_g);  // 判断设备是否静止
    float acc_alpha = static_state ? 0.8f : 0.5f;           // 静止时滤波系数更大（平滑效果更好）

// ===================== 4. 加速度计低通滤波 =====================
    lowpass_filter(&ax_g, &ay_g, &az_g,
                   &imu.acc.filtered[X],
                   &imu.acc.filtered[Y],
                   &imu.acc.filtered[Z],
                   acc_alpha);

// 取滤波后的加速度数据
    float ax = imu.acc.filtered[X];
    float ay = imu.acc.filtered[Y];
    float az = imu.acc.filtered[Z]; 

// 加速度数据归一化
    normalize(&ax, &ay, &az);

// ===================== 5. 读取当前四元数 =====================
// 取出当前四元数 (w, x, y, z)
    float q0 = imu.quaternion.qua[0];
    float q1 = imu.quaternion.qua[1];
    float q2 = imu.quaternion.qua[2];
    float q3 = imu.quaternion.qua[3];

// ===================== 6. 计算重力向量在机体坐标系的投影 =====================
// 根据当前四元数计算重力向量在机体坐标系中的投影（用于与加速度计数据对比）
    float gx = 2 * (q1 * q3 - q0 * q2);
    float gy = 2 * (q0 * q1 + q2 * q3);
    float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

// ===================== 7. 计算梯度误差（加速度计与重力投影的误差） =====================
    float ex = ay * gz - az * gy;
    float ey = az * gx - ax * gz;
    float ez = ax * gy - ay * gx;

// ===================== 8. 自适应调整PI校正系数 =====================
// 根据静止状态调整校正系数（运动时减弱校正，避免引入噪声）KI不衰减，保证积分校正力度
    float kp = static_state ? imu_kp : imu_kp * 0.8f;
    float ki = imu_ki;

// 首次计算的前 0.1 秒，使用大比例系数加速收敛
    if (first_count_time < 0.1f)
    {
        first_count_time += delta_T;  // 累加时间
        kp = 10.0f;  // 强比例校正，快速收敛
    }
    
// 动态时积分力度降为1/10
    float integral_gain = static_state ? 1.0f : 0.1f;

// ===================== 9. 积分误差累计与限幅 =====================
    imu.acc.error[X] += (ex * delta_T) * integral_gain;
    imu.acc.error[Y] += (ey * delta_T) * integral_gain;
    imu.acc.error[Z] += (ez * delta_T) * integral_gain;

// 积分限幅: 防止饱和
    float acc_err_limit = 1.0f;
    imu.acc.error[X] =func_limit_ab(imu.acc.error[X], -acc_err_limit, acc_err_limit);
    imu.acc.error[Y] =func_limit_ab(imu.acc.error[Y], -acc_err_limit, acc_err_limit);
    imu.acc.error[Z] =func_limit_ab(imu.acc.error[Z], -acc_err_limit, acc_err_limit);
   
// ===================== 10. 用误差校正陀螺仪数据（比例+积分校正） =====================
    x += kp * ex + ki * imu.acc.error[X];
    y += kp * ey + ki * imu.acc.error[Y];
    z += kp * ez + ki * imu.acc.error[Z];

// ===================== 11. 四元数微分方程更新（根据陀螺仪角速度） =====================
    q0 += ((-q1 * x - q2 * y - q3 * z) * delta_T / 2.0f);
    q1 += (( q0 * x + q2 * z - q3 * y) * delta_T / 2.0f);              
    q2 += (( q0 * y - q1 * z + q3 * x) * delta_T / 2.0f);
    q3 += (( q0 * z + q1 * y - q2 * x) * delta_T / 2.0f);

// ===================== 12. 四元数归一化（避免数值漂移） =====================

// 四元数归一化（避免数值漂移）
    length = sqrt(q0*q0 +q1*q1 +q2*q2 + q3*q3  );  // 计算模长

    if (length > 0.001f)  // 模长有效时才归一化
    {
        q0 /= length;
        q1 /= length;
        q2 /= length;
        q3 /= length;
    }
// 更新四元数 (w, x, y, z)
    imu.quaternion.qua[0] = q0;
    imu.quaternion.qua[1] = q1;
    imu.quaternion.qua[2] = q2;
    imu.quaternion.qua[3] = q3;


// ===================== 13. 根据四元数计算旋转矩阵（用于后续 ===================== 
float q0_2=q0*q0;
float q1_2=q1*q1;
float q2_2=q2*q2;
float q3_2=q3*q3;

 float g1 = 2.0f*(q1 * q3 - q0 * q2);
 float g2 = 2.0f*(q0 * q1 + q2 * q3);
 float g3 =q0_2-q1_2-q2_2+q3_2;
 float g4 = 2.0f*(q1 * q2 + q0 * q3);
 float g5 =q0_2+q1_2-q2_2-q3_2;

// ===================== 14. 根据旋转矩阵计算姿态角（横滚角、俯仰角、偏航角） =====================                                           // 俯仰角

    eulerAngle.yaw = atan2(g4, g5)*57.3f;  // 偏航角 
    eulerAngle.pitch = -asin(g1)*57.3f;  
    eulerAngle.roll = atan2(g2, g3)*57.3f;  // 横滚角

                                           // 俯仰角

}
float arctan1(float x)
{
   float angle=(func_abs(x)>1.0f)?
   90.0f-func_abs(1.0f/x)*(45.0f-(func_abs(1.0f/x)-1.0f)*(14.0f+3.83f*func_abs(1.0f/x) )):
   func_abs(x)*(45.0f-(func_abs(x)-1.0f)*(14.0f+3.83f*func_abs(x)));
   return(x>0.0f)?angle:-angle;
 
}
float arctan2(float y,float x)
{
    float angle;
    if(x==0.0f&&y==0.0f) return 0;
    if(x==0.0f)
    {
        if(y>0.0f) return  90.0f;
        else return -90.0f; 
    }
    if(y==0.0f)
    {
        if(x>0.0f) return  0.0f;
        else return -180.0f; 
    }
    
 angle=arctan1(y/x);
    
    if(x<0.0f&&angle>0.0f) angle-=180.0f;
    else if(x<0.0f&&angle<0.0f) angle+=180.0f;
    return angle;
}
float arcsin(float x)
{
   return arctan1(x/sqrt(1.0f-x*x));
}
