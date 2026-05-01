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
********************************************************************************************************************/

#include "camera_service.h"

//====================================================全局变量定义====================================================
camera_data_struct  camera_data;                // 摄像头数据结构体
track_info_struct   track_info;                 // 赛道信息结构体
camera_task_info_struct camera_task_info;       // 摄像头任务语义输出
uint8               binary_threshold;           // 当前二值化阈值

static uint8 blue_zone_detect_hold_limit = 2;
static uint8 blue_zone_release_hold_limit = 1;
static uint8 mine_box_detect_hold_limit = 2;
static uint8 mine_box_release_hold_limit = 2;
static uint8 cone_gap_detect_hold_limit = 1;
static uint8 cone_gap_release_hold_limit = 1;
static uint8 bridge_detect_hold_limit = 2;
static uint8 bridge_release_hold_limit = 2;
static uint8 stairs_detect_hold_limit = 2;
static uint8 stairs_release_hold_limit = 1;

// 内部缓冲区
static uint8 binary_image[CAMERA_IMAGE_H][CAMERA_IMAGE_W];  // 二值化图像缓冲区

// Otsu算法内部缓冲区（直方图，256级灰度）
static uint16 histogram[256];                   // 灰度直方图

// 边沿跟踪种子点
static uint8 seed_left;                         // 左边界种子点
static uint8 seed_right;                        // 右边界种子点

// 任务语义滞回变量
static camera_task_info_struct camera_task_candidate;
static uint8 blue_zone_hold_count  = 0;
static uint8 mine_box_hold_count   = 0;
static uint8 cone_gap_hold_count   = 0;
static uint8 bridge_hold_count     = 0;
static uint8 stairs_hold_count     = 0;
//====================================================全局变量定义====================================================


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头服务模块初始化
// 参数说明     void
// 返回参数     uint8           0-成功 非0-失败
// 使用示例     camera_service_init();
// 备注信息     初始化总钻风摄像头（MT9V03X），使用中断+DMA模式采集，不占用CPU
//-------------------------------------------------------------------------------------------------------------------
uint8 camera_service_init(void)
{
    uint8 init_retry = 0;
    
    // 初始化二值化阈值为默认值（后续由Otsu动态计算）
    binary_threshold = CAMERA_THRESHOLD_DEF;
    
    // 初始化种子点为图像中心
    seed_left  = CAMERA_IMAGE_CENTER;
    seed_right = CAMERA_IMAGE_CENTER;
    
    // 初始化总钻风摄像头，失败则重试
    // mt9v03x_init() 内部配置了DMA和中断回调，采集完成后会设置 mt9v03x_finish_flag
    while(mt9v03x_init())
    {
        system_delay_ms(500);                   // 初始化失败延时重试
        init_retry++;
        if(init_retry > 5)                      // 重试5次后放弃
        {
            return 1;                           // 返回初始化失败
        }
    }
    
    // 初始化摄像头数据结构体
    camera_data.image_ptr = mt9v03x_image[0];   // 直接指向缓冲区（指针操作，避免内存拷贝）
    camera_data.width     = CAMERA_IMAGE_W;
    camera_data.height    = CAMERA_IMAGE_H;
    camera_data.is_ready  = 0;
    
    // 初始化赛道信息结构体
    memset(&track_info, 0, sizeof(track_info_struct));
    track_info.offset         = 0;
    track_info.error          = 0;
    track_info.road_type      = ROAD_STRAIGHT;
    track_info.is_valid       = 0;
    track_info.otsu_threshold = CAMERA_THRESHOLD_DEF;

    // 初始化任务语义结构体
    memset(&camera_task_info, 0, sizeof(camera_task_info_struct));
    memset(&camera_task_candidate, 0, sizeof(camera_task_info_struct));
    blue_zone_hold_count = 0;
    mine_box_hold_count = 0;
    cone_gap_hold_count = 0;
    bridge_hold_count = 0;
    stairs_hold_count = 0;

    return 0;                                   // 返回初始化成功
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Otsu大津法动态阈值计算（整数定点运算版本）
// 参数说明     img_data        图像数据指针
// 参数说明     width           图像宽度
// 参数说明     height          图像高度（仅统计中部60行）
// 返回参数     uint8           计算得到的最佳阈值
// 使用示例     threshold = camera_calculate_otsu_threshold(mt9v03x_image[0], 188, 120);
// 备注信息     【核心优化1】使用整数运算，避免float
//             仅统计中部60行（OTSU_CALC_START_ROW ~ OTSU_CALC_END_ROW）提高效率
//             原理：最大化类间方差 = w0 * w1 * (u0 - u1)^2
//-------------------------------------------------------------------------------------------------------------------
uint8 camera_calculate_otsu_threshold(uint8 *img_data, uint16 width, uint16 height)
{
    uint32 pixel_total = 0;                     // 像素总数
    uint32 gray_sum = 0;                        // 灰度值总和
    uint32 gray_sum_bg = 0;                     // 背景灰度值累加
    uint32 pixel_bg = 0;                        // 背景像素数
    uint32 pixel_fg = 0;                        // 前景像素数
    uint32 variance_max = 0;                    // 最大类间方差（放大1000000倍）
    uint8  threshold_best = CAMERA_THRESHOLD_DEF;
    uint16 row, col;
    uint8  *row_ptr;
    uint8  gray_val;
    int32  mean_diff;                           // 均值差（放大1000倍）
    uint32 variance_between;                    // 当前类间方差
    
    // 清空直方图
    memset(histogram, 0, sizeof(histogram));
    
    // 【优化】仅统计图像中部60行，提高计算效率
    for(row = OTSU_CALC_START_ROW; row < OTSU_CALC_END_ROW; row++)
    {
        row_ptr = &mt9v03x_image[row][0];       // 指针直接访问，避免memcpy
        for(col = 0; col < width; col++)
        {
            gray_val = row_ptr[col];
            histogram[gray_val]++;
            gray_sum += gray_val;
            pixel_total++;
        }
    }
    
    // 防止除零
    if(pixel_total == 0)
    {
        return CAMERA_THRESHOLD_DEF;
    }
    
    // 遍历所有可能的阈值，寻找最大类间方差
    for(uint16 t = 1; t < 255; t++)
    {
        pixel_bg += histogram[t - 1];
        gray_sum_bg += (uint32)(t - 1) * histogram[t - 1];
        
        pixel_fg = pixel_total - pixel_bg;
        
        // 跳过空类
        if(pixel_bg == 0 || pixel_fg == 0)
        {
            continue;
        }
        
        // 计算类间方差（使用定点数运算，放大避免精度损失）
        // variance = w0 * w1 * (u0 - u1)^2
        // 其中：u0 = gray_sum_bg / pixel_bg, u1 = (gray_sum - gray_sum_bg) / pixel_fg
        // 为避免除法精度损失，变换公式：
        // mean_diff = (gray_sum_bg * pixel_fg - (gray_sum - gray_sum_bg) * pixel_bg) / (pixel_bg * pixel_fg)
        // 放大1000倍计算
        
        int32 numerator = (int32)gray_sum_bg * (int32)pixel_fg - 
                          (int32)(gray_sum - gray_sum_bg) * (int32)pixel_bg;
        
        // 使用简化公式：variance ∝ (pixel_bg * pixel_fg * (u0 - u1)^2)
        // 直接比较 numerator^2 / (pixel_bg * pixel_fg) 的大小
        // 为避免溢出，分步计算
        
        uint32 denom = pixel_bg * pixel_fg;
        if(denom == 0) continue;
        
        // 计算 |numerator| / sqrt(denom) 的近似比较
        // 简化：比较 numerator^2 / denom
        int64 num_sq = (int64)numerator * numerator;
        variance_between = (uint32)(num_sq / denom);
        
        if(variance_between > variance_max)
        {
            variance_max = variance_between;
            threshold_best = (uint8)t;
        }
    }
    
    // 阈值范围限制（防止极端光照条件）
    if(threshold_best < CAMERA_THRESHOLD_MIN)
    {
        threshold_best = CAMERA_THRESHOLD_MIN;
    }
    else if(threshold_best > CAMERA_THRESHOLD_MAX)
    {
        threshold_best = CAMERA_THRESHOLD_MAX;
    }
    
    return threshold_best;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头帧处理（主循环调用）
// 参数说明     void
// 返回参数     void
// 使用示例     camera_process_frame();
// 备注信息     检测图像采集完成标志，执行完整的图像处理流程（V2.0 竞赛级算法）
//             底层使用中断回调（camera_finish_callback）设置完成标志，不占用CPU
//-------------------------------------------------------------------------------------------------------------------
void camera_process_frame(void)
{
    camera_data.is_ready = 1;                    // 标记数据就绪

    //==================== V2.0 智能车竞赛级处理流程 ====================

    // 1. Otsu动态阈值计算
    binary_threshold = camera_calculate_otsu_threshold(
        mt9v03x_image[0], CAMERA_IMAGE_W, CAMERA_IMAGE_H);
    track_info.otsu_threshold = binary_threshold;

    // 2. 二值化处理（使用动态阈值）
    camera_image_binarize();

    // 3. 边沿跟踪+滑动窗口算法
    camera_find_edge();

    // 4. 越野环境滤噪
    camera_filter_noise();

    // 5. 计算赛道中线
    camera_calc_center_line();

    // 6. 计算偏移量
    camera_get_offset();

    // 7. 更新赛道类型状态机
    camera_update_road_type();

    // 8. 更新任务语义输出
    camera_update_task_semantics();
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像二值化处理
// 参数说明     void
// 返回参数     void
// 使用示例     camera_image_binarize();
// 备注信息     将灰度图转换为黑白图，使用Otsu动态阈值
//             白色(255)表示赛道，黑色(0)表示赛道外区域
//-------------------------------------------------------------------------------------------------------------------
void camera_image_binarize(void)
{
    uint8  *img_ptr;                            // 图像行指针
    uint8  *bin_ptr;                            // 二值化行指针
    int16  row, col;
    uint8  thresh = binary_threshold;           // 本地缓存阈值，提高访问效率
    
    // 从下往上扫描（因为下方图像更重要）
    for(row = CAMERA_SCAN_START_ROW; row >= CAMERA_SCAN_END_ROW; row--)
    {
        img_ptr = &mt9v03x_image[row][0];       // 指向原始图像当前行
        bin_ptr = &binary_image[row][0];        // 指向二值化图像当前行
        
        // 逐列处理（可考虑4字节对齐优化）
        for(col = 0; col < CAMERA_IMAGE_W; col++)
        {
            // 大于阈值为白色（赛道），小于等于阈值为黑色（赛道外）
            bin_ptr[col] = (img_ptr[col] > thresh) ? 255 : 0;
        }
        
        // 防止 row 减到负数（row 是 uint16 类型）
        if(row == CAMERA_SCAN_END_ROW) break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     边沿跟踪+滑动窗口算法（V2.0核心算法）
// 参数说明     void
// 返回参数     void
// 使用示例     camera_find_edge();
// 备注信息     【核心优化2】废弃"每行从中心搜索"的低效逻辑
//             基于"种子点"的边沿搜索：
//             - 从底行中点向左右找到初始边界
//             - 随后每行基于上一行边界位置，在±15像素范围内建立滑动窗口局部搜索
//             - 丢失边界时使用前两行斜率进行线性补偿
//-------------------------------------------------------------------------------------------------------------------
void camera_find_edge(void)
{
    uint8  *bin_ptr;                            // 二值化图像行指针
    uint16 row;
    int16  col;
    uint8  search_start, search_end;
    uint8  left_found, right_found;
    
    // 边界跟踪的前两行记录（用于斜率补偿）
    int16  left_prev1, left_prev2;              // 前一行、前两行的左边界
    int16  right_prev1, right_prev2;            // 前一行、前两行的右边界
    int16  slope_left, slope_right;             // 边界斜率
    
    // 统计变量
    uint32 width_sum = 0;
    uint8  valid_count = 0;
    
    // 初始化
    track_info.left_lost_cnt  = 0;
    track_info.right_lost_cnt = 0;
    memset(track_info.row_trust, 0, sizeof(track_info.row_trust));
    
    //==================== 第一步：底行初始化种子点 ====================
    row = CAMERA_SCAN_START_ROW;
    bin_ptr = &binary_image[row][0];
    
    // 从图像中心向左搜索，找到第一个白->黑跳变点作为左种子
    seed_left = 0;
    for(col = CAMERA_IMAGE_CENTER; col > 0; col--)
    {
        if((bin_ptr[col] == 255) && (bin_ptr[col - 1] == 0))
        {
            seed_left = (uint8)col;
            break;
        }
    }
    
    // 从图像中心向右搜索，找到第一个白->黑跳变点作为右种子
    seed_right = CAMERA_IMAGE_W - 1;
    for(col = CAMERA_IMAGE_CENTER; col < CAMERA_IMAGE_W - 1; col++)
    {
        if((bin_ptr[col] == 255) && (bin_ptr[col + 1] == 0))
        {
            seed_right = (uint8)col;
            break;
        }
    }
    
    // 记录底行边界
    track_info.left_edge[row]  = seed_left;
    track_info.right_edge[row] = seed_right;
    
    if(seed_right > seed_left)
    {
        width_sum += (seed_right - seed_left);
        valid_count++;
    }
    
    // 初始化前行记录
    left_prev1 = seed_left;
    left_prev2 = seed_left;
    right_prev1 = seed_right;
    right_prev2 = seed_right;
    
    //==================== 第二步：逐行边沿跟踪（滑动窗口） ====================
    for(row = CAMERA_SCAN_START_ROW - 1; row >= CAMERA_SCAN_END_ROW; row--)
    {
        bin_ptr = &binary_image[row][0];
        left_found  = 0;
        right_found = 0;
        
        // 初始化为无效值
        track_info.left_edge[row]  = CAMERA_EDGE_INVALID;
        track_info.right_edge[row] = CAMERA_EDGE_INVALID;
        
        //---------- 左边界滑动窗口搜索 ----------
        // 计算搜索窗口：以前一行左边界为中心，±EDGE_SEARCH_WINDOW范围
        search_start = (left_prev1 > EDGE_SEARCH_WINDOW) ? (left_prev1 - EDGE_SEARCH_WINDOW) : 0;
        search_end   = (left_prev1 + EDGE_SEARCH_WINDOW < CAMERA_IMAGE_W) ? 
                       (left_prev1 + EDGE_SEARCH_WINDOW) : (CAMERA_IMAGE_W - 1);
        
        // 在窗口内搜索白->黑跳变点
        for(col = search_end; col > search_start; col--)
        {
            if((bin_ptr[col] == 255) && (bin_ptr[col - 1] == 0))
            {
                track_info.left_edge[row] = (uint8)col;
                left_found = 1;
                break;
            }
        }
        
        // 如果窗口内没找到，检查窗口边界
        if(!left_found)
        {
            if(search_start == 0 && bin_ptr[0] == 0)
            {
                track_info.left_edge[row] = 0;
                left_found = 1;
            }
        }
        
        // 【核心】丢失边界时使用斜率补偿
        if(!left_found && EDGE_PREDICT_ENABLE)
        {
            slope_left = left_prev1 - left_prev2;  // 斜率 = 当前趋势
            
            // 限制最大斜率，防止预测跑飞
            if(slope_left > EDGE_PREDICT_MAX_SLOPE) slope_left = EDGE_PREDICT_MAX_SLOPE;
            if(slope_left < -EDGE_PREDICT_MAX_SLOPE) slope_left = -EDGE_PREDICT_MAX_SLOPE;
            
            // 线性外推预测
            int16 predict_left = left_prev1 + slope_left;
            if(predict_left < 0) predict_left = 0;
            if(predict_left >= CAMERA_IMAGE_W) predict_left = CAMERA_IMAGE_W - 1;
            
            track_info.left_edge[row] = (uint8)predict_left;
            track_info.left_lost_cnt++;
            track_info.row_trust[row] |= CAMERA_ROW_UNTRUST;  // 标记为不可信
        }
        
        //---------- 右边界滑动窗口搜索 ----------
        search_start = (right_prev1 > EDGE_SEARCH_WINDOW) ? (right_prev1 - EDGE_SEARCH_WINDOW) : 0;
        search_end   = (right_prev1 + EDGE_SEARCH_WINDOW < CAMERA_IMAGE_W) ? 
                       (right_prev1 + EDGE_SEARCH_WINDOW) : (CAMERA_IMAGE_W - 1);
        
        for(col = search_start; col < search_end; col++)
        {
            if((bin_ptr[col] == 255) && (bin_ptr[col + 1] == 0))
            {
                track_info.right_edge[row] = (uint8)col;
                right_found = 1;
                break;
            }
        }
        
        if(!right_found)
        {
            if(search_end >= CAMERA_IMAGE_W - 1 && bin_ptr[CAMERA_IMAGE_W - 1] == 0)
            {
                track_info.right_edge[row] = CAMERA_IMAGE_W - 1;
                right_found = 1;
            }
        }
        
        // 右边界斜率补偿
        if(!right_found && EDGE_PREDICT_ENABLE)
        {
            slope_right = right_prev1 - right_prev2;
            
            if(slope_right > EDGE_PREDICT_MAX_SLOPE) slope_right = EDGE_PREDICT_MAX_SLOPE;
            if(slope_right < -EDGE_PREDICT_MAX_SLOPE) slope_right = -EDGE_PREDICT_MAX_SLOPE;
            
            int16 predict_right = right_prev1 + slope_right;
            if(predict_right < 0) predict_right = 0;
            if(predict_right >= CAMERA_IMAGE_W) predict_right = CAMERA_IMAGE_W - 1;
            
            track_info.right_edge[row] = (uint8)predict_right;
            track_info.right_lost_cnt++;
            track_info.row_trust[row] |= CAMERA_ROW_UNTRUST;
        }
        
        //---------- 更新前行记录 ----------
        left_prev2  = left_prev1;
        left_prev1  = (track_info.left_edge[row] != CAMERA_EDGE_INVALID) ? 
                      track_info.left_edge[row] : left_prev1;
        right_prev2 = right_prev1;
        right_prev1 = (track_info.right_edge[row] != CAMERA_EDGE_INVALID) ? 
                      track_info.right_edge[row] : right_prev1;
        
        //---------- 统计赛道宽度 ----------
        if(track_info.left_edge[row] != CAMERA_EDGE_INVALID && 
           track_info.right_edge[row] != CAMERA_EDGE_INVALID &&
           track_info.right_edge[row] > track_info.left_edge[row])
        {
            uint8 width = track_info.right_edge[row] - track_info.left_edge[row];
            width_sum += width;
            valid_count++;
        }
        
        // 防止 row 减到负数
        if(row == CAMERA_SCAN_END_ROW) break;
    }
    
    // 计算平均赛道宽度
    track_info.road_width_avg = (valid_count > 0) ? (uint8)(width_sum / valid_count) : 0;
    track_info.valid_row_count = valid_count;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     越野环境滤噪
// 参数说明     void
// 返回参数     void
// 使用示例     camera_filter_noise();
// 备注信息     【核心优化3】增加越野环境鲁棒性
//             1. 边界平滑：如果某行边界与相邻行跳变超过阈值，视为噪点，取邻值填充
//             2. 赛道宽度检查：如果宽度偏差过大，标记该行为"不可信数据"
//-------------------------------------------------------------------------------------------------------------------
void camera_filter_noise(void)
{
    uint16 row;
    int16  left_diff, right_diff;
    uint8  width_current;
    int16  width_diff;
    uint8  avg_width = track_info.road_width_avg;
    
    // 防止除零
    if(avg_width < WIDTH_MIN_VALID)
    {
        avg_width = WIDTH_MIN_VALID;
    }
    
    // 从下往上遍历（跳过底行）
    for(row = CAMERA_SCAN_START_ROW - 1; row >= CAMERA_SCAN_END_ROW + 1; row--)
    {
        //==================== 边界跳变滤噪 ====================
        // 检查左边界跳变
        if(track_info.left_edge[row] != CAMERA_EDGE_INVALID && 
           track_info.left_edge[row + 1] != CAMERA_EDGE_INVALID)
        {
            left_diff = (int16)track_info.left_edge[row] - (int16)track_info.left_edge[row + 1];
            
            // 如果跳变超过阈值，视为噪点
            if(left_diff > NOISE_JUMP_THRESHOLD || left_diff < -NOISE_JUMP_THRESHOLD)
            {
                // 取前一行值填充（向下看是 row+1）
                track_info.left_edge[row] = track_info.left_edge[row + 1];
                track_info.row_trust[row] |= CAMERA_ROW_UNTRUST;
            }
        }
        
        // 检查右边界跳变
        if(track_info.right_edge[row] != CAMERA_EDGE_INVALID && 
           track_info.right_edge[row + 1] != CAMERA_EDGE_INVALID)
        {
            right_diff = (int16)track_info.right_edge[row] - (int16)track_info.right_edge[row + 1];
            
            if(right_diff > NOISE_JUMP_THRESHOLD || right_diff < -NOISE_JUMP_THRESHOLD)
            {
                track_info.right_edge[row] = track_info.right_edge[row + 1];
                track_info.row_trust[row] |= CAMERA_ROW_UNTRUST;
            }
        }
        
        //==================== 赛道宽度检查 ====================
        if(track_info.left_edge[row] != CAMERA_EDGE_INVALID && 
           track_info.right_edge[row] != CAMERA_EDGE_INVALID &&
           track_info.right_edge[row] > track_info.left_edge[row])
        {
            width_current = track_info.right_edge[row] - track_info.left_edge[row];
            
            // 宽度合法性检查
            if(width_current < WIDTH_MIN_VALID || width_current > WIDTH_MAX_VALID)
            {
                track_info.row_trust[row] |= CAMERA_ROW_UNTRUST;
            }
            
            // 与平均宽度偏差检查（使用整数运算：偏差百分比 = |diff| * 100 / avg）
            width_diff = (int16)width_current - (int16)avg_width;
            if(width_diff < 0) width_diff = -width_diff;
            
            // 偏差超过 WIDTH_DEVIATION_RATIO% 标记为不可信
            if((uint16)width_diff * 100 > (uint16)avg_width * WIDTH_DEVIATION_RATIO)
            {
                track_info.row_trust[row] |= CAMERA_ROW_UNTRUST;
            }
        }
        
        // 防止 row 减到负数
        if(row == CAMERA_SCAN_END_ROW + 1) break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算赛道中线
// 参数说明     void
// 返回参数     void
// 使用示例     camera_calc_center_line();
// 备注信息     根据滤噪后的左右边界计算每一行的赛道中心点
//-------------------------------------------------------------------------------------------------------------------
void camera_calc_center_line(void)
{
    uint16 row;
    uint8  left, right;
    
    // 计算每一行的中线
    for(row = CAMERA_SCAN_START_ROW; row >= CAMERA_SCAN_END_ROW; row--)
    {
        left  = track_info.left_edge[row];
        right = track_info.right_edge[row];
        
        // 检查边界有效性
        if((left != CAMERA_EDGE_INVALID) && (right != CAMERA_EDGE_INVALID) && (right > left))
        {
            // 中线 = (左边界 + 右边界) / 2
            track_info.center_line[row] = (uint8)((left + right) >> 1);  // 位运算替代除法
        }
        else
        {
            // 边界无效时，中线默认为图像中心
            track_info.center_line[row] = CAMERA_IMAGE_CENTER;
        }
        
        // 防止 row 减到负数
        if(row == CAMERA_SCAN_END_ROW) break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取赛道中线偏移量
// 参数说明     void
// 返回参数     int16           偏移量（正值：车辆偏右，需左转；负值：车辆偏左，需右转）
// 使用示例     offset = camera_get_offset();
// 备注信息     计算加权平均偏移量，近处权重大，远处权重小
//             只使用"可信"行的数据，不可信行不参与计算
//-------------------------------------------------------------------------------------------------------------------
int16 camera_get_offset(void)
{
    uint16 row;
    int32  weighted_sum = 0;                    // 加权偏差累加（使用int32防止溢出）
    int32  weight_total = 0;                    // 权重累加
    int16  center_offset;                       // 当前行偏差
    uint16 weight;                              // 当前行权重
    uint8  trust_count = 0;                     // 可信行计数
    
    // 同时计算中线斜率（用于弯道识别）
    int32  slope_sum = 0;
    int16  prev_center = CAMERA_IMAGE_CENTER;
    uint8  slope_count = 0;
    
    // 从下往上扫描，下方（近处）权重大
    for(row = CAMERA_SCAN_START_ROW; row >= CAMERA_SCAN_END_ROW; row--)
    {
        // 只使用可信行的数据
        if(track_info.row_trust[row] == 0 &&
           track_info.left_edge[row] != CAMERA_EDGE_INVALID && 
           track_info.right_edge[row] != CAMERA_EDGE_INVALID)
        {
            // 计算当前行的偏差（中线位置 - 图像中心）
            center_offset = (int16)track_info.center_line[row] - (int16)CAMERA_IMAGE_CENTER;
            
            // 权重计算：距离车辆越近（row越大），权重越大
            weight = (uint16)(row - CAMERA_SCAN_END_ROW + 1);
            
            // 累加加权偏差
            weighted_sum += (int32)center_offset * (int32)weight;
            weight_total += weight;
            trust_count++;
            
            // 计算中线斜率（相邻行中线差值的累加）
            if(slope_count > 0)
            {
                slope_sum += ((int16)track_info.center_line[row] - prev_center);
            }
            prev_center = track_info.center_line[row];
            slope_count++;
        }
        
        // 防止 row 减到负数
        if(row == CAMERA_SCAN_END_ROW) break;
    }
    
    // 计算中线斜率（放大100倍，整数运算）
    if(slope_count > 1)
    {
        track_info.center_slope = (int16)((slope_sum * 100) / (slope_count - 1));
    }
    else
    {
        track_info.center_slope = 0;
    }
    
    // 计算加权平均偏移量
    if(weight_total > 0 && trust_count >= MIN_VALID_ROWS)
    {
        track_info.offset   = (int16)(weighted_sum / weight_total);
        track_info.error    = track_info.offset;    // error 与 offset 相同，供PID使用
        track_info.is_valid = 1;                    // 标记数据有效
    }
    else
    {
        track_info.offset   = 0;
        track_info.error    = 0;
        track_info.is_valid = 0;                    // 标记数据无效
    }
    
    return track_info.offset;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新赛道类型状态机
// 参数说明     void
// 返回参数     void
// 使用示例     camera_update_road_type();
// 备注信息     【核心优化4】根据有效行数和中心线斜率变化，实时更新当前赛道类型
//             赛道类型：STRAIGHT/CURVE_L/CURVE_R/LOST/OBSTACLE
//-------------------------------------------------------------------------------------------------------------------
void camera_update_road_type(void)
{
    int16 slope_abs;
    
    // 丢线判断（有效行数过少）
    if(track_info.valid_row_count < LOST_VALID_ROWS_THRESHOLD || !track_info.is_valid)
    {
        track_info.road_type = ROAD_LOST;
        return;
    }
    
    // 计算斜率绝对值
    slope_abs = track_info.center_slope;
    if(slope_abs < 0) slope_abs = -slope_abs;
    
    // 根据斜率判断弯道/直道
    if(slope_abs <= STRAIGHT_SLOPE_THRESHOLD * 100)
    {
        // 斜率很小，直道
        track_info.road_type = ROAD_STRAIGHT;
    }
    else if(slope_abs >= CURVE_SLOPE_THRESHOLD * 100)
    {
        // 斜率较大，弯道
        if(track_info.center_slope > 0)
        {
            // 中线向右倾斜，左弯道
            track_info.road_type = ROAD_CURVE_L;
        }
        else
        {
            // 中线向左倾斜，右弯道
            track_info.road_type = ROAD_CURVE_R;
        }
    }
    else
    {
        // 中等斜率，保持当前状态或默认直道
        // 可根据实际情况细化判断逻辑
        if(track_info.road_type == ROAD_LOST)
        {
            track_info.road_type = ROAD_STRAIGHT;
        }
    }
    
    // 障碍物检测（可扩展：检测赛道宽度突变）
    // TODO: 根据实际需求添加障碍物检测逻辑
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置二值化阈值（手动模式）
// 参数说明     thresh          阈值 (0-255)
// 返回参数     void
// 使用示例     camera_set_threshold(120);
// 备注信息     可临时覆盖Otsu自动阈值，用于调试
//-------------------------------------------------------------------------------------------------------------------
void camera_set_threshold(uint8 thresh)
{
    binary_threshold = thresh;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前二值化阈值
// 参数说明     void
// 返回参数     uint8           当前阈值（可能是Otsu计算值或手动设置值）
// 使用示例     current_thresh = camera_get_threshold();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 camera_get_threshold(void)
{
    return binary_threshold;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取中线斜率
// 参数说明     void
// 返回参数     int16           斜率值（放大100倍的定点数）
// 使用示例     slope = camera_get_center_slope();
// 备注信息     正值表示中线向右倾斜（左弯），负值表示向左倾斜（右弯）
//-------------------------------------------------------------------------------------------------------------------
int16 camera_get_center_slope(void)
{
    return track_info.center_slope;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前赛道类型
// 参数说明     void
// 返回参数     road_type_enum  当前赛道类型
// 使用示例     type = camera_get_road_type();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
road_type_enum camera_get_road_type(void)
{
    return track_info.road_type;
}

void camera_update_task_semantics(void)
{
    memset(&camera_task_candidate, 0, sizeof(camera_task_info_struct));

    if(track_info.is_valid && track_info.valid_row_count > 35 && track_info.road_width_avg > 120)
    {
        if(track_info.otsu_threshold < 95)
        {
            camera_task_candidate.blue_zone_detected = 1;
            camera_task_candidate.stairs_detected = 1;
            camera_task_candidate.stairs_center_offset = track_info.offset;
        }
    }

    if(track_info.is_valid && track_info.valid_row_count > 45 &&
       track_info.road_width_avg > 130 && track_info.road_width_avg < 180)
    {
        if(track_info.offset > -20 && track_info.offset < 20)
        {
            camera_task_candidate.mine_box_detected = 1;
            camera_task_candidate.mine_box_offset = track_info.offset;
        }
    }

    if(track_info.is_valid && track_info.valid_row_count > 40 &&
       track_info.road_width_avg < 120 &&
       track_info.left_lost_cnt < 25 && track_info.right_lost_cnt < 25)
    {
        camera_task_candidate.bridge_detected = 1;
        camera_task_candidate.bridge_center_offset = track_info.offset;
    }

    if(track_info.is_valid && track_info.valid_row_count > 30 &&
       track_info.road_width_avg > 100)
    {
        if(track_info.offset < -15)
        {
            camera_task_candidate.cone_gap_detected = 1;
            camera_task_candidate.cone_gap_offset = -25;
        }
        else if(track_info.offset > 15)
        {
            camera_task_candidate.cone_gap_detected = 1;
            camera_task_candidate.cone_gap_offset = 25;
        }
        else if(track_info.center_slope > 150 || track_info.center_slope < -150)
        {
            camera_task_candidate.cone_gap_detected = 1;
            camera_task_candidate.cone_gap_offset = 0;
        }
    }

    if(camera_task_candidate.blue_zone_detected)
    {
        if(blue_zone_hold_count < blue_zone_detect_hold_limit + blue_zone_release_hold_limit)
        {
            blue_zone_hold_count++;
        }
    }
    else if(blue_zone_hold_count > 0)
    {
        blue_zone_hold_count--;
    }

    if(camera_task_candidate.mine_box_detected)
    {
        if(mine_box_hold_count < mine_box_detect_hold_limit + mine_box_release_hold_limit)
        {
            mine_box_hold_count++;
        }
        camera_task_info.mine_box_offset = camera_task_candidate.mine_box_offset;
    }
    else if(mine_box_hold_count > 0)
    {
        mine_box_hold_count--;
    }

    if(camera_task_candidate.bridge_detected)
    {
        if(bridge_hold_count < bridge_detect_hold_limit + bridge_release_hold_limit)
        {
            bridge_hold_count++;
        }
        camera_task_info.bridge_center_offset = camera_task_candidate.bridge_center_offset;
    }
    else if(bridge_hold_count > 0)
    {
        bridge_hold_count--;
    }

    if(camera_task_candidate.stairs_detected)
    {
        if(stairs_hold_count < stairs_detect_hold_limit + stairs_release_hold_limit)
        {
            stairs_hold_count++;
        }
        camera_task_info.stairs_center_offset = camera_task_candidate.stairs_center_offset;
    }
    else if(stairs_hold_count > 0)
    {
        stairs_hold_count--;
    }

    if(camera_task_candidate.cone_gap_detected)
    {
        if(cone_gap_hold_count < cone_gap_detect_hold_limit + cone_gap_release_hold_limit)
        {
            cone_gap_hold_count++;
        }
        camera_task_info.cone_gap_offset = camera_task_candidate.cone_gap_offset;
    }
    else if(cone_gap_hold_count > 0)
    {
        cone_gap_hold_count--;
    }

    camera_task_info.blue_zone_hold_frames = blue_zone_hold_count;
    camera_task_info.mine_box_hold_frames = mine_box_hold_count;
    camera_task_info.cone_gap_hold_frames = cone_gap_hold_count;
    camera_task_info.bridge_hold_frames = bridge_hold_count;
    camera_task_info.stairs_hold_frames = stairs_hold_count;

    camera_task_info.blue_zone_detected = (blue_zone_hold_count >= blue_zone_detect_hold_limit);
    camera_task_info.mine_box_detected = (mine_box_hold_count >= mine_box_detect_hold_limit);
    camera_task_info.bridge_detected = (bridge_hold_count >= bridge_detect_hold_limit);
    camera_task_info.stairs_detected = (stairs_hold_count >= stairs_detect_hold_limit);
    camera_task_info.cone_gap_detected = (cone_gap_hold_count >= cone_gap_detect_hold_limit);

    if(!camera_task_info.mine_box_detected)
    {
        camera_task_info.mine_box_offset = 0;
    }
    if(!camera_task_info.bridge_detected)
    {
        camera_task_info.bridge_center_offset = 0;
    }
    if(!camera_task_info.stairs_detected)
    {
        camera_task_info.stairs_center_offset = 0;
    }
    if(!camera_task_info.cone_gap_detected)
    {
        camera_task_info.cone_gap_offset = 0;
    }
}
