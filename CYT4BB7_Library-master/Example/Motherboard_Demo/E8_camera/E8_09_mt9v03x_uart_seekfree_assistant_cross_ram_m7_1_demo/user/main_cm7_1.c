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
* 文件名称          main_cm7_1
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

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

#pragma location = 0x2802bf80                                                   // 将下面这个数组定义到指定的RAM地址(只会强制指定下面的一个定义)
__no_init uint8 image_copy[MT9V03X_H][MT9V03X_W];

#pragma location = 0x28037c55
__no_init uint8 image_fresh_flag;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
     
    // 此处编写用户代码 例如外设初始化代码等

    
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);           // 设置逐飞助手使用DEBUG串口进行收发
    
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);              // 配置发送总钻风图像信息(仅包含原始图像信息)
    

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
        
        SCB_CleanInvalidateDCache_by_Addr(&image_fresh_flag, 1);                // 访问标志位之前，刷新同步RAM数据
        if(image_fresh_flag == 1)
        {
            SCB_CleanInvalidateDCache_by_Addr(image_copy[0], MT9V03X_IMAGE_SIZE);  // 访问备份数组之前，刷新同步RAM数据
            seekfree_assistant_camera_send();                                   // 发送图像
            
            image_fresh_flag = 0;
            SCB_CleanInvalidateDCache_by_Addr(&image_fresh_flag, 1);            // 写入标志位之后，刷新同步RAM数据
        }
        
        system_delay_ms(1);                                                     // 增加主循环延时 防止长时间占用标志位访问
            
      
      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
