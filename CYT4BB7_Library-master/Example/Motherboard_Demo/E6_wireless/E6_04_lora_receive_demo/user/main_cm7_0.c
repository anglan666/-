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
// 接入lora无线模块
//      模块管脚            单片机管脚
//      RX                  P04_1
//      TX                  P04_0
//      GND                 主板电源地 GND
//      5V                  主板 5V 电源

//================================特别注意================================
// 串口接线时一定要接GND 否则无法正常通讯
//================================特别注意================================
//================================特别注意================================
// 串口接线时一定要接GND 否则无法正常通讯
//================================特别注意================================
//================================特别注意================================
// 串口接线时一定要接GND 否则无法正常通讯
//================================特别注意================================

// ***************************** 例程测试说明 *****************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，在断电情况下完成连接
// 2.将LORA无线模块连接到主板，切记查看引脚是否对应
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.当LORA无线模块接收到对应数据时，可以在串口助手上看到如下串口信息：
//      lora_receive_num:0,last_num:0
//      lora_receive_num:0,last_num:0
//      lora_receive_num:0,last_num:0
//      lora_receive_num:1,last_num:0
//      lora_receive_num:1,last_num:1
//      lora_receive_num:1,last_num:1
//      lora_receive_num:2,last_num:1
//      lora_receive_num:2,last_num:2
//      lora_receive_num:2,last_num:2
//      lora_receive_num:2,last_num:2
//      lora_receive_num:3,last_num:2
//      lora_receive_num:3,last_num:3
//      lora_receive_num:3,last_num:3
//      lora_receive_num:3,last_num:3
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#define LORA_UART_INDEX            (UART_1		  )             
#define LORA_UART_BUAD_RATE        (115200		  )
#define LORA_UART_TX_PIN           (UART1_RX_P04_0        )
#define LORA_UART_RX_PIN           (UART1_TX_P04_1        )

uint8 lora_get_data[4];                 // 接收数据缓冲数组                               
uint8 lora_get_num = 0;                 
uint8 lora_receive_num;                 // 当前收到的序号
uint8 lora_receive_num_last;            // 上一次收到的序号

void lora_communication_receive(void)
{
    uint8 temp_data;
    if(uart_query_byte(WIRELESS_UART_INDEX, &temp_data))
    {
          if(temp_data == 0x66)
          {
              lora_get_num = 0;
          }
          lora_get_data[lora_get_num] = temp_data;
          if(lora_get_data[lora_get_num] == 0x88 && lora_get_num == 3)
          {
              lora_receive_num_last = lora_receive_num;
              lora_receive_num = lora_get_data[1];
              lora_get_num = 0;
          }
          lora_get_num ++;
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    
    // 此处编写用户代码 例如外设初始化代码等
    
    set_wireless_type(WIRELESS_UART, lora_communication_receive);                               // 设置回调函数
    
    uart_init (LORA_UART_INDEX, LORA_UART_BUAD_RATE, LORA_UART_RX_PIN, LORA_UART_TX_PIN);       // 初始化LORA串口
    
    uart_rx_interrupt(LORA_UART_INDEX, 1);                                                      // 打开串口接收中断    
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码


      printf("lora_receive_num:%d,last_num:%d\r\n", lora_receive_num, lora_receive_num_last);   // 打印接收数据
      
      system_delay_ms(100);
      
      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
// **************************** 例程常见问题说明 ****************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看串口助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率