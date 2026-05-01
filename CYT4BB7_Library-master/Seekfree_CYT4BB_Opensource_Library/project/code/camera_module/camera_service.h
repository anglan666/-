/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          camera_service
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-01-24       user               first version
* 2024-01-25       user               v2.0 智能车竞赛级算法重构
*                                     - Otsu动态阈值
*                                     - 边沿跟踪+滑动窗口
*                                     - 越野环境滤噪
*                                     - 赛道状态机
********************************************************************************************************************/

#ifndef _CAMERA_SERVICE_H_
#define _CAMERA_SERVICE_H_

#include "zf_common_headfile.h"

//====================================================摄像头服务模块配置====================================================
// 本模块仅负责图像采集与赛道提取，不涉及其他业务逻辑（严禁出现任何GPS相关代码）
// 算法版本：V2.0 智能车竞赛级
//====================================================摄像头服务模块配置====================================================

//====================================================图像参数定义====================================================
// 图像尺寸（引用底层驱动定义，确保一致性）
#define CAMERA_IMAGE_W              MT9V03X_W                   // 图像宽度 188
#define CAMERA_IMAGE_H              MT9V03X_H                   // 图像高度 120
#define CAMERA_IMAGE_SIZE           MT9V03X_IMAGE_SIZE          // 图像总大小 188*120
#define CAMERA_IMAGE_CENTER         (CAMERA_IMAGE_W / 2)        // 图像中心列 94

// 扫描区域参数
#define CAMERA_SCAN_START_ROW       (CAMERA_IMAGE_H - 1)        // 扫描起始行 119（从图像底部开始）
#define CAMERA_SCAN_END_ROW         (10)                        // 扫描结束行（图像顶部附近）
#define CAMERA_SCAN_ROW_COUNT       (CAMERA_SCAN_START_ROW - CAMERA_SCAN_END_ROW + 1)

// Otsu算法参数（仅统计中部区域提高效率）
#define OTSU_CALC_START_ROW         (30)                        // Otsu统计起始行
#define OTSU_CALC_END_ROW           (90)                        // Otsu统计结束行（共60行）
#define OTSU_CALC_ROW_COUNT         (OTSU_CALC_END_ROW - OTSU_CALC_START_ROW)
#define CAMERA_THRESHOLD_MIN        (30)                        // 最小阈值限制（防止过暗）
#define CAMERA_THRESHOLD_MAX        (220)                       // 最大阈值限制（防止过亮）
#define CAMERA_THRESHOLD_DEF        (100)                       // 默认阈值（备用）

// 边沿跟踪参数（滑动窗口算法）
#define EDGE_SEARCH_WINDOW          (15)                        // 滑动窗口半宽（±15像素）
#define EDGE_PREDICT_ENABLE         (1)                         // 启用斜率预测补偿
#define EDGE_PREDICT_MAX_SLOPE      (5)                         // 最大预测斜率（像素/行）

// 滤噪与鲁棒性参数
#define NOISE_JUMP_THRESHOLD        (10)                        // 边界跳变噪点阈值（像素）
#define WIDTH_DEVIATION_RATIO       (40)                        // 宽度偏差比例阈值（百分比，40%）
#define WIDTH_MIN_VALID             (20)                        // 最小有效赛道宽度（像素）
#define WIDTH_MAX_VALID             (180)                       // 最大有效赛道宽度（像素）
#define MIN_VALID_ROWS              (15)                        // 最小有效行数

// 赛道类型判断参数
#define CURVE_SLOPE_THRESHOLD       (3)                         // 弯道斜率阈值（定点数，放大100倍）
#define STRAIGHT_SLOPE_THRESHOLD    (1)                         // 直道斜率阈值
#define LOST_VALID_ROWS_THRESHOLD   (10)                        // 丢线判断阈值（有效行数）

// 边界标记值
#define CAMERA_EDGE_INVALID         (255)                       // 无效边界值
#define CAMERA_ROW_UNTRUST          (0x01)                      // 行数据不可信标记
//====================================================图像参数定义====================================================

//====================================================赛道类型枚举====================================================
typedef enum
{
    ROAD_STRAIGHT   = 0,                        // 直道
    ROAD_CURVE_L    = 1,                        // 左弯道
    ROAD_CURVE_R    = 2,                        // 右弯道
    ROAD_LOST       = 3,                        // 丢线/脱轨
    ROAD_OBSTACLE   = 4,                        // 障碍物
    ROAD_CROSS      = 5,                        // 十字路口
} road_type_enum;
//====================================================赛道类型枚举====================================================

// 摄像头任务语义结构体（面向比赛任务）
typedef struct
{
    uint8   blue_zone_detected;                  // 是否检测到蓝色任务区域
    uint8   blue_zone_hold_frames;               // 蓝色区域保持计数
    uint8   mine_box_detected;                   // 是否检测到白色雷区边框
    uint8   mine_box_hold_frames;                // 雷区边框保持计数
    int16   mine_box_offset;                     // 雷区中心偏移
    uint8   cone_gap_detected;                   // 是否检测到锥桶间隙
    uint8   cone_gap_hold_frames;                // 锥桶间隙保持计数
    int16   cone_gap_offset;                     // 锥桶间隙中心偏移
    uint8   bridge_detected;                     // 是否检测到单边桥/窄通道
    uint8   bridge_hold_frames;                  // 单边桥保持计数
    int16   bridge_center_offset;                // 单边桥中心偏移
    uint8   stairs_detected;                     // 是否检测到台阶/坡道结构
    uint8   stairs_hold_frames;                  // 台阶保持计数
    int16   stairs_center_offset;                // 台阶中心偏移
} camera_task_info_struct;

// 摄像头数据结构体 - 供决策层通过接口获取数据
typedef struct
{
    uint8   *image_ptr;                         // 指向图像缓冲区的指针（避免大规模内存拷贝）
    uint16  width;                              // 图像宽度
    uint16  height;                             // 图像高度
    vuint8  is_ready;                           // 图像就绪标志 1:就绪 0:未就绪
} camera_data_struct;

// 赛道信息结构体 - 存储赛道提取结果（智能车竞赛级）
typedef struct
{
    // 边界与中线数据
    uint8   left_edge[CAMERA_IMAGE_H];          // 左边界数组（每行一个值）
    uint8   right_edge[CAMERA_IMAGE_H];         // 右边界数组（每行一个值）
    uint8   center_line[CAMERA_IMAGE_H];        // 中线数组（每行一个值）
    uint8   row_trust[CAMERA_IMAGE_H];          // 行数据可信度标记（0:可信 非0:不可信）
    
    // 核心输出
    int16   offset;                             // 赛道中线偏移量（正值右偏，负值左偏）
    int16   error;                              // 加权偏差值（用于PID控制）
    
    // 赛道特征
    uint8   road_width_avg;                     // 平均赛道宽度
    uint8   valid_row_count;                    // 有效行计数
    int16   center_slope;                       // 中线斜率（定点数，放大100倍）
    
    // 赛道状态机
    road_type_enum  road_type;                  // 当前赛道类型
    uint8   is_valid;                           // 赛道数据有效标志
    
    // 调试信息
    uint8   otsu_threshold;                     // 当前Otsu计算的阈值
    uint8   left_lost_cnt;                      // 左边界丢失行数
    uint8   right_lost_cnt;                     // 右边界丢失行数
} track_info_struct;
//====================================================数据结构定义====================================================

//====================================================全局变量声明====================================================
extern camera_data_struct   camera_data;        // 摄像头数据（供外部模块读取）
extern track_info_struct    track_info;         // 赛道信息（供决策层使用）
extern camera_task_info_struct camera_task_info; // 摄像头任务语义输出
extern uint8                binary_threshold;   // 当前二值化阈值
//====================================================全局变量声明====================================================

//====================================================函数声明====================================================
// 核心接口函数
uint8 camera_service_init       (void);         // 摄像头服务初始化
void  camera_process_frame      (void);         // 摄像头帧处理（主循环调用）

// Otsu动态阈值算法
uint8 camera_calculate_otsu_threshold(uint8 *img_data, uint16 width, uint16 height);

// 图像处理函数
void  camera_image_binarize     (void);         // 图像二值化处理（使用动态阈值）
void  camera_find_edge          (void);         // 边沿跟踪+滑动窗口算法
void  camera_filter_noise       (void);         // 越野环境滤噪
void  camera_calc_center_line   (void);         // 计算赛道中线
int16 camera_get_offset         (void);         // 获取赛道中线偏移量
void  camera_update_road_type   (void);         // 更新赛道类型状态机

// 辅助函数
void  camera_set_threshold      (uint8 thresh); // 手动设置二值化阈值
uint8 camera_get_threshold      (void);         // 获取当前二值化阈值
int16 camera_get_center_slope   (void);         // 获取中线斜率
road_type_enum camera_get_road_type(void);      // 获取当前赛道类型
void  camera_update_task_semantics(void);       // 更新比赛任务语义输出
//====================================================函数声明====================================================

#endif
