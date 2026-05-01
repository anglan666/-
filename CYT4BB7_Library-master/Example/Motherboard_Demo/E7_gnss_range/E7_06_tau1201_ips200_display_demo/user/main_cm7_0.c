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
// 接入gnss TAU1201连接
//      模块管脚            单片机管脚
//      RX                  查看 zf_device_gnss.h 文件中 gnss_RX 宏定义的引脚 
//      TX                  查看 zf_device_gnss.h 文件中 gnss_TX 宏定义的引脚 
//      GND                 电源地
//      VCC                 5V电源
// 接入IPS2.0屏幕
//      模块管脚            单片机管脚
//      BL                  查看 zf_device_ips200_parallel8.h 中 IPS200_BL_PIN 宏定义  
//      CS                  查看 zf_device_ips200_parallel8.h 中 IPS200_CS_PIN 宏定义  
//      RST                 查看 zf_device_ips200_parallel8.h 中 IPS200_RST_PIN 宏定义 
//      RS                  查看 zf_device_ips200_parallel8.h 中 IPS200_RS_PIN 宏定义  
//      WR                  查看 zf_device_ips200_parallel8.h 中 IPS200_WR_PIN 宏定义  
//      RD                  查看 zf_device_ips200_parallel8.h 中 IPS200_RD_PIN 宏定义  
//      D0-D7               查看 zf_device_ips200_parallel8.h 中 IPS200_Dx_PIN 宏定义  
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源




// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 接好gnss 连接好屏幕
// 2.在断电情况下完成连接 完成上电
// 3.屏幕会显示当前gnss收到的数据解析结果
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#define IPS200_TYPE     (IPS200_TYPE_PARALLEL8)                                 // 八位并口 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
                                                                                // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI

uint8 no_data_count = 0;
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    
    // 此处编写用户代码 例如外设初始化代码等

    gnss_init(TAU1201);
    ips200_init(IPS200_TYPE);

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        //gnss数据接收与解析都是通过串口中断调用gnss_uart_callback函数进行实现的
        //数据解析完毕之后gnss_flag标志位会置1
        if(gnss_flag)
        {
            gnss_flag = 0;

            gnss_data_parse();           //开始解析数据

            ips200_show_uint(   0, 16*0, gnss.time.year, 4);
            ips200_show_uint(  80, 16*0, gnss.time.month, 2);
            ips200_show_uint( 160, 16*0, gnss.time.day, 2);
            ips200_show_uint(   0, 16*1, gnss.time.hour, 2);
            ips200_show_uint(  80, 16*1, gnss.time.minute, 2);
            ips200_show_uint( 160, 16*1, gnss.time.second, 2);

            ips200_show_uint(   0, 16*2, gnss.state, 5);
            ips200_show_float(120, 16*2, gnss.latitude, 4, 6);
            ips200_show_float(  0, 16*3, gnss.longitude, 4, 6);
            ips200_show_float(120, 16*3, gnss.speed, 4, 6);
            ips200_show_float(  0, 16*4, gnss.direction, 4, 6);
            ips200_show_uint( 120, 16*4, gnss.satellite_used, 5);
            ips200_show_float(  0, 16*5, gnss.height, 4, 6);
        }
        else
        {
            no_data_count ++;
        }

        if(no_data_count > 3)
        {
            zf_log(0, "GPS data was not received");
        }
        system_delay_ms(1000);//这里延时主要目的是为了降低输出速度，便于在串口助手中观察数据，实际使用的时候不需要这样去延时

        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
//      检查一下gnss模块接线是否正常
// 问题2：数据显示除了时间以外全部是0
//      需要把天线放到室外 需要接收到卫星信号才能有数据
