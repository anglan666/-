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
//      直接将调试下载器正确连接在核心板的调试下载接口即可
// 接入硅麦采集模块
//      模块管脚            单片机管脚
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//      OUT1                P12_5
//      OUT2                P14_4
// 
// ***************************** 例程测试说明 *****************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 3.电脑上使用逐飞助手打开对应的串口，串口波特率为 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在逐飞助手上看到如下串口信息：
//      -181.0,-162.0
//      -185.0,-165.0
//      -174.0,-158.0
//      -182.0,-168.0
//      -187.0,-160.0
//      -185.0,-170.0
//      ......
// 5.可以在逐飞助手的示波器界面看到FFT运算完成后的结果波形
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查
// **************************** 代码区域 ****************************

#define MIC_PIN_1         ADC1_CH09_P12_5       // 硅麦1信号采集接口
#define MIC_PIN_2         ADC1_CH24_P14_4       // 硅麦2信号采集接口

// 需要注意 FFT_SIZE 的大小并不是随意设置，而是与采集频率有关系。
// 我们需要保证采集 FFT_SIZE 个点所对应的时间刚好与信号时一样长的，由于信号长度为0.2048秒，
// 本例程采集样频率为10khz，意味着采样间距为0.1ms，采样2048个点也就是0.2048秒刚好与信号时间是一样的。
// 自己修改的时候需要特别注意
#define FFT_SIZE          2048                  // 用于FFT计算的数据长度
#define MIC_RAW_DATA_LEN  2500                  // 用于采集硅麦的循环数组长度(需要大于 FFT_SIZE 防止数据异常)

int16 mic_raw_data[2][MIC_RAW_DATA_LEN];        // 定义采集需要用到的循环数组
int16 mic_raw_data_count;                       // 定义循环数组计数位
int16 mic_raw_data_count_save;                  // 定义循环数组计数位-保存值(用于取出最近的 FFT_SIZE 个数据)
 
float fft_signal[2][2*FFT_SIZE];                // 定义取出数据的保存位置

void mic_data_copy(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    
    // 此处编写用户代码 例如外设初始化代码等
    
    // 初始化麦克风采集端口
    adc_init(MIC_PIN_1, ADC_12BIT);
    adc_init(MIC_PIN_2, ADC_12BIT);  
    
    // 使用定时器的0通道   创建一个周期频率为10K的周期中断
    pit_us_init(PIT_CH0, 100);
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
 
        mic_data_copy();        // 读取最近的 FFT_SIZE 个麦克风数据
        
        for(int i = 0; i < FFT_SIZE; i ++)      // 输出最近的 FFT_SIZE 个数据
        {
            // 若需要进行后续 FFT 运算 可参考开源库的FFT例程
            // 此处仅输出采集信号的实部 用于验证采集正确
            printf("%.1f,%.1f\r\n", fft_signal[0][i * 2], fft_signal[1][i * 2]);
        }
        
        system_delay_ms(1000);   // 延时防止串口数据频繁更新
      
        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     采集麦克风数据
// 参数说明     void
// 返回参数     void   
// 使用示例     mic_data_get();
// 备注信息     该函数在中断中调用
//-------------------------------------------------------------------------------------------------------------------
void mic_data_get (void)
{
    // 因为在做FFT计算的时候我们需要减去直流偏量，因此这里固定减2048，实际应该自己通过测量没有声音时的数值来确定这里应该减去多少
    // 当没有声音的时候，减去偏置后得到的数值应该是在0附近，说明减的偏置就是正确的
    mic_raw_data[0][mic_raw_data_count] = adc_convert(MIC_PIN_1) - 2048;
    mic_raw_data[1][mic_raw_data_count] = adc_convert(MIC_PIN_2) - 2048;

    // 采集点数加一
    mic_raw_data_count++;

    if(mic_raw_data_count >= 2500)       // 判断当前采集点数是否已经达到数组末尾
    {
        mic_raw_data_count = 0;          // 采集次数清零  实现循环
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复制硅麦数据(最近的 FFT_SIZE 个)
// 参数说明     void
// 返回参数     void   
// 使用示例     mic_data_copy();
// 备注信息     取出循环数组最近的2048个数据用于互相关计算
//-------------------------------------------------------------------------------------------------------------------
void mic_data_copy(void)
{
    uint16 mic_raw_data_count_temp = 0;         
    uint16 mic_data_count = 0;
    
    mic_raw_data_count_save = mic_raw_data_count;       // 保存当前时刻的采集位置( mic_raw_data_count 会在中断被持续更新，保存以防止数据异常）
    
    if(mic_raw_data_count_save < FFT_SIZE)              // 若当前采集位置小于需复制的长度(无法一个循环完成复制)，则分为两端分别复制数据
    {
        mic_raw_data_count_temp = MIC_RAW_DATA_LEN - (FFT_SIZE - mic_raw_data_count_save);      
        
        for(int16 i = mic_raw_data_count_temp;i < MIC_RAW_DATA_LEN; i ++)
        {
            fft_signal[0][mic_data_count * 2] = mic_raw_data[0][i];             // 保存用于FFT计算的数据 实部赋值
            fft_signal[1][mic_data_count * 2] = mic_raw_data[1][i];
            fft_signal[0][mic_data_count * 2 + 1] = 0;                          // 保存用于FFT计算的数据 虚部赋值
            fft_signal[1][mic_data_count * 2 + 1] = 0;
            mic_data_count ++;
        }
        for(int16 i = 0;i < mic_raw_data_count_save; i ++)
        {
            fft_signal[0][mic_data_count * 2] = mic_raw_data[0][i];             // 保存用于FFT计算的数据 实部赋值
            fft_signal[1][mic_data_count * 2] = mic_raw_data[1][i];
            fft_signal[0][mic_data_count * 2 + 1] = 0;
            fft_signal[1][mic_data_count * 2 + 1] = 0;                          // 保存用于FFT计算的数据 虚部赋值
            mic_data_count ++;
        }
    }
    else                                                                        // 若当前采集位置大于需复制的长度，则直接一个循环完成复制
    {
        for(int16 i = 0;i < FFT_SIZE; i ++)
        {
            fft_signal[0][i * 2] = mic_raw_data[0][i];                          // 保存用于FFT计算的数据 实部赋值
            fft_signal[1][i * 2] = mic_raw_data[1][i];
            fft_signal[0][i * 2 + 1] = 0;                                       // 保存用于FFT计算的数据 虚部赋值
            fft_signal[1][i * 2 + 1] = 0;       
        }
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
