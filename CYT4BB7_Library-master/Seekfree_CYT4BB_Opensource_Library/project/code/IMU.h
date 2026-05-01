
#ifndef IMU_H_
#define IMU_H_

#include "zf_common_headfile.h"

//===================================================宏定义===================================================
/**
 * @defgroup IMU_Config 配置宏定义
 * @brief IMU模块的配置参数
 * @{
 */

#define delta_T  0.005f                 /*!< 采样周期 5ms，与姿态解算频率匹配 */


/** @} */

/**
 * @defgroup IMU_Sensitivity 传感器灵敏度定义
 * @brief 根据IMU类型定义不同的灵敏度参数
 */

#if IMU_TYPE
    #define GYRO_SPL 14.3               /*!< 陀螺仪灵敏度 (imu963ra)，单位：LSB/(°/s) */
    #define ACC_SPL  4098.0             /*!< 加速度计灵敏度 (imu963ra)，单位：LSB/g */
    #define MAG_SPL  3000.0             /*!< 磁力计灵敏度 (imu963ra)，单位：LSB/mT */

    #define GYRO_DATA_X        ( imu963ra_gyro_x)
    #define GYRO_DATA_Y        ( imu963ra_gyro_y)
    #define GYRO_DATA_Z        ( imu963ra_gyro_z)

    #define ACC_DATA_X         ( imu963ra_acc_x)
    #define ACC_DATA_Y         ( imu963ra_acc_y)
    #define ACC_DATA_Z         ( imu963ra_acc_z)
    
    #define MAG_DATA_X         ( imu963ra_mag_x)
    #define MAG_DATA_Y         ( imu963ra_mag_y)
    #define MAG_DATA_Z         ( imu963ra_mag_z)

#else
    #define GYRO_SPL 16.4               /*!< 陀螺仪灵敏度 (imu660ra)，单位：LSB/(°/s) */
    #define ACC_SPL  4096.0             /*!< 加速度计灵敏度 (imu660ra)，单位：LSB/g */
    #define GYRO_DATA_X        (imu660ra_gyro_x)
    #define GYRO_DATA_Y        (imu660ra_gyro_y)
    #define GYRO_DATA_Z        (imu660ra_gyro_z)

    #define ACC_DATA_X         (imu660ra_acc_x)
    #define ACC_DATA_Y         (imu660ra_acc_y)
    #define ACC_DATA_Z         (imu660ra_acc_z)

#endif


typedef enum
{
    X,                                  /*!< X轴，索引值为0 */
    Y,                                  /*!< Y轴，索引值为1 */
    Z,                                  /*!< Z轴，索引值为2 */
    NUM_XYZ                             /*!< 轴向总数，用于数组大小定义 */
}imu_info_enum;


typedef struct
{
    double pitch;                       /*!< 俯仰角 (Pitch)，绕X轴旋转，范围：-180~180度 */
    double roll;                        /*!< 横滚角 (Roll)，绕Y轴旋转，范围：-90~90度 */
    double yaw;                         /*!< 偏航角 (Yaw)，绕Z轴旋转，范围：0~360度 */
}eulerAngle_info_struct;
/**
 * @brief IMU信息结构体
 * @details 整合加速度计和陀螺仪的数据和状态，是核心数据结构
 * @note 包含原始数据、处理后的数据、校准状态和角度信息
 */
typedef struct {
    struct {
        float acc[3];                   /*!< 加速度数据，单位：g（重力加速度） */
        float angle[3];                 /*!< 加速度计算的角度，单位：度 */
        float filtered[3];              /*!< 加速度计低通滤波后的数据，单位：g（重力加速度） */
        float error[3];                 /*!< 加速度计误差，单位：g（重力加速度） */
        float offset[3];                /*!< 加速度计偏移，单位：g（重力加速度） */
    } acc;                              /*!< 加速度计数据结构 */
    struct {
        float gyro[3];                  /*!< 陀螺仪数据，单位：rad/s（弧度/秒） */
        float angle[3];                 /*!< 陀螺仪积分的角度，单位：度 */
        float zero_angle[3];            /*!< 零点角度值，校准过程中累加的偏移 */
        float last_angle[3];            /*!< 上一次角度值，用于计算角度变化量 */
        float offset[3];                /*!< 陀螺仪偏移，单位：rad/s（弧度/秒） */
    } gyro;                             /*!< 陀螺仪数据结构 */
    struct {
        float qua[4];          
        float rot_mat[3][3];              /*!< 旋转矩阵，用于将四元数转换为欧拉角 */
    } quaternion;                        /*!< 四元数数据结构 */
} imu_info_struct;

extern imu_info_struct imu;                                 /*!< IMU数据结构体实例，包含所有传感器数据 */
extern eulerAngle_info_struct eulerAngle;                   /*!< 欧拉角结构体实例，存储解算后的姿态 */

void IMU_init(void);

/**
 * @brief 获取传感器原始数据
 * @details 从IMU传感器读取加速度计和陀螺仪的原始数据
 *          根据IMU_TYPE宏自动选择对应的传感器驱动
 * @note 建议在1ms中断中调用，保证数据更新频率
 * @warning 此函数只读取数据，不进行任何处理
 */
void IMU_get_data(void);
void IMU_Zero_Bias_correction(void);
void IMU_data_processing(void);
void IMU_update_angle(void);
float arctan1(float x);
float arctan2(float y,float x);
float arcsin(float x);
/** @} */

#endif /* IMU_H_ */
