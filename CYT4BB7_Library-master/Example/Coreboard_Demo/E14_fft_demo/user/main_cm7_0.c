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
#include "arm_math.h"

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

// ***************************** 例程测试说明 *****************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 3.电脑上使用逐飞助手打开对应的串口，串口波特率为 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在逐飞助手上看到如下串口信息：
// fft count use time: 876 us
//
// ifft count use time: 964 us
// 5.可以在逐飞助手的示波器界面看到FFT运算完成后的结果波形
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

#define FFT_SIZE        (2048 )                  // FFT数据大小
#define SAMPLING_FREQ   (10000)                  // 信号采样频率
#define FFT_DEMO_PI     (3.1415926)              

float inputSignal[FFT_SIZE * 2];                // 定义输入信号 输入信号为复数 所以长度为 FFT_SIZE * 2
float fft_outputSignal[FFT_SIZE * 2];           // 定义FFT输出信号 输出信号为复数 所以长度为 FFT_SIZE * 2

uint32 fft_count_time_us  = 0;
uint32 ifft_count_time_us = 0;


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	        // 时钟配置及系统初始化<务必保留>
    debug_info_init();                          // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    // **************************** 信号预处理 ****************************
    for(int i = 0; i < FFT_SIZE; i++){
        // 对输入数据虚拟赋值，这里模拟输入一个500hz的正弦波信号
        // 实际声音信号由ADC采集
        // 将输入信号填入实部，虚部为0(FFT计算使用复数，float数组的复数存储方式为实部虚部交替存储)
        inputSignal[2 * i] = arm_sin_f32(FFT_DEMO_PI * i / 10);
        inputSignal[2*i + 1] = 0;
    }    
    // **************************** 信号预处理 ****************************
    
    
    // **************************** 初始化内容 ****************************
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);           // 初始化逐飞助手组件 选择debug串口输出信息
    
    seekfree_assistant_oscilloscope_data.channel_num  = 2;                      // 配置通道长度为1组
    
    arm_cfft_instance_f32 arm_cfft_instance_f32_len_2048;                       // 定义FFT对象
          
    arm_cfft_init_f32(&arm_cfft_instance_f32_len_2048, FFT_SIZE);               // 初始化FFT对象 赋予计算长度
    
    timer_init(TC_TIME2_CH0, TIMER_US);                                         // 初始化一个定时器 用于记录FFT运算的耗时
    // **************************** 初始化内容 ****************************
    
    
    // ***************************** FFT运算 *****************************  
    timer_start(TC_TIME2_CH0);                                                  // 启动定时器
    
    arm_cfft_f32(&arm_cfft_instance_f32_len_2048 , inputSignal , 0 , 1);        // 32位浮点FFT运算(运算结果将替换原inputSignal的内容)
    
    fft_count_time_us = timer_get(TC_TIME2_CH0);                                // 获取FFT运算时长
    
    timer_clear(TC_TIME2_CH0);                                                  // 清除定时器计数值
    
    timer_stop(TC_TIME2_CH0);                                                   // 关闭定时器

    arm_cmplx_mag_f32(inputSignal , fft_outputSignal , FFT_SIZE);               // 将FFT结果转换为幅度谱(转换幅度谱是为了观察波形)
    // ***************************** FFT运算 *****************************  
    
    
    // **************************** 逆FFT运算 **************************** 
    timer_start(TC_TIME2_CH0);                                                  // 启动定时器
    
    arm_cfft_f32(&arm_cfft_instance_f32_len_2048 , inputSignal , 1 , 1);        // 32位浮点逆FFT运算(运算结果将替换原inputSignal的内容)
    
    ifft_count_time_us = timer_get(TC_TIME2_CH0);                               // 获取逆FFT运算时长
    
    timer_clear(TC_TIME2_CH0);                                                  // 清除定时器计数值
    
    timer_stop(TC_TIME2_CH0);                                                   // 关闭定时器
    // **************************** 逆FFT运算 **************************** 
    
    
    // ************************** 计算结果输出 *************************** 
    for(int i = 0; i < FFT_SIZE; i++)
    {
        seekfree_assistant_oscilloscope_data.data[0] = fft_outputSignal[i];     // 获取逆FFT运算后的幅度信息
        seekfree_assistant_oscilloscope_data.data[1] = inputSignal[i * 2];      // 获取逆FFT运算后的幅度信息
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);     // 输出幅度信息到示波器                                     
    }   
    
    printf("\r\n fft count use time: %d us\r\n",   fft_count_time_us);          // 输出FFT运算耗时
    printf("\r\n ifft count use time: %d us\r\n", ifft_count_time_us);          // 输出逆FFT运算耗时
    // ************************** 计算结果输出 *************************** 
    
    
    // ****************************** 总结 ******************************* 
    // 可以通过逐飞助手的示波器观察到FFT计算结果中，在数组下标为 103 的位置的值最大，此时反推模拟的输入信号频率
    // 采样频率为10khz，FFT计算长度为2048个数据，那么FFT频率点则为 10000 除以 2048 等于 4.8828125 hz，意味着FFT计算结果的间隔为4.8828125 hz
    // 因此数组下标为 103 的位置的频率则是 103 * 4.8828125 = 502.9296875 hz，此时可以大概得出原信号频率(没有准确计算出来500hz是因为有计算精度误差)
    // ****************************** 总结 ******************************* 

    
    // 此处编写用户代码 例如外设初始化代码等
    
    while(true)
    {
        // 此处编写需要循环执行的代码


        
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************

// **************************** 例程常见问题说明 ****************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看逐飞助手打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看逐飞助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
