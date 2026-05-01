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
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚 默认 P14_0
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚 默认 P14_1
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源
// 接入正交编码器连接
//      模块管脚            单片机管脚
//      A                   ENCODER_QUADDEC_A 宏定义的引脚 默认 B4
//      B                   ENCODER_QUADDEC_B 宏定义的引脚 默认 B5
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// 接入方向编码器连接
//      模块管脚            单片机管脚
//      LSB                 ENCODER_DIR_PULSE 宏定义的引脚 默认 B6
//      DIR                 ENCODER_DIR_DIR 宏定义的引脚 默认 B7
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源



// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好编码器，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在串口助手上看到如下串口信息：
//      ENCODER_QUADDEC counter     x .
//      ENCODER_DIR counter         x .
// 5.转动编码器就会看到数值变化
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

#define PIT0                             (PIT_CH0 )                             // 使用的周期中断编号

#define ENCODER_DIR1                     (TC_CH58_ENCODER)                      // 编码器接口       
#define ENCODER_DIR1_PULSE               (TC_CH58_ENCODER_CH1_P17_3)            // PULSE 对应的引脚                      
#define ENCODER_DIR1_DIR                 (TC_CH58_ENCODER_CH2_P17_4)            // DIR 对应的引脚                        
                                                                                
#define ENCODER_DIR2                     (TC_CH27_ENCODER)                      // 编码器接口   
#define ENCODER_DIR2_PULSE               (TC_CH27_ENCODER_CH1_P19_2)            // PULSE 对应的引脚                  
#define ENCODER_DIR2_DIR                 (TC_CH27_ENCODER_CH2_P19_3)            // DIR 对应的引脚                    
                                                                                
#define ENCODER_QUAD1                    (TC_CH07_ENCODER)                      // 编码器接口  
#define ENCODER_QUAD1_PHASE_A            (TC_CH07_ENCODER_CH1_P07_6)            // PHASE_A 对应的引脚                 
#define ENCODER_QUAD1_PHASE_B            (TC_CH07_ENCODER_CH2_P07_7)            // PHASE_B 对应的引脚                   
                                                                                
#define ENCODER_QUAD2                    (TC_CH20_ENCODER)                      // 编码器接口
#define ENCODER_QUAD2_PHASE_A            (TC_CH20_ENCODER_CH1_P08_1)            // PHASE_A 对应的引脚
#define ENCODER_QUAD2_PHASE_B            (TC_CH20_ENCODER_CH2_P08_2)            // PHASE_B 对应的引脚

int16 encoder_data_dir[2] = {0};
int16 encoder_data_quad[2] = {0};

uint8 pit_state = 0;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等


    encoder_dir_init (ENCODER_DIR1,  ENCODER_DIR1_PULSE,    ENCODER_DIR1_DIR);       // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init (ENCODER_DIR2,  ENCODER_DIR2_PULSE,    ENCODER_DIR2_DIR);       // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_quad_init(ENCODER_QUAD1, ENCODER_QUAD1_PHASE_A, ENCODER_QUAD1_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD2, ENCODER_QUAD2_PHASE_A, ENCODER_QUAD2_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式

    pit_ms_init(PIT0, 100);                                                     // 初始化 PIT0 为周期中断 100ms 周
    
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
        if(pit_state)
        {
            pit_state = 0;
            encoder_data_dir[0] = encoder_get_count(ENCODER_DIR1);
            encoder_clear_count(ENCODER_DIR1);
            encoder_data_dir[1] = encoder_get_count(ENCODER_DIR2);
            encoder_clear_count(ENCODER_DIR2);
            encoder_data_quad[0] = encoder_get_count(ENCODER_QUAD1);
            encoder_clear_count(ENCODER_QUAD1);
            encoder_data_quad[1] = encoder_get_count(ENCODER_QUAD2);
            encoder_clear_count(ENCODER_QUAD2);
            
            printf("ENCODER_DIR_1 counter \t\t%d .\r\n", encoder_data_dir[0]);      // 输出编码器计数信息
            printf("ENCODER_DIR_2 counter \t\t%d .\r\n", encoder_data_dir[1]);      // 输出编码器计数信息
            printf("ENCODER_QUAD_3 counter \t\t%d .\r\n", encoder_data_quad[0]);      // 输出编码器计数信息
            printf("ENCODER_QUAD_4 counter \t\t%d .\r\n", encoder_data_quad[1]);      // 输出编码器计数信息
        }

        
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看串口助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：数值一直在正负一跳转
//      不要把方向编码器接在正交编码器模式的接口下
// 问题4：数值不随编码转动变化
//      如果使用主板测试，主板必须要用电池供电
//      检查编码器是否是正常的，线是否松动，编码器是否发热烧了，是否接错线
