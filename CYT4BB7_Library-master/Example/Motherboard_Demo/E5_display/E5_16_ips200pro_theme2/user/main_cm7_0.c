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

#include "ipspro_page_ctrl.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 接入两寸ips200pro屏幕
//      模块管脚            单片机管脚
//      CLK                 查看 zf_device_ips200pro.h 中 IPS200PRO_CLK_PIN  宏定义  默认 P12_2
//      MOSI                查看 zf_device_ips200pro.h 中 IPS200PRO_MOSI_PIN 宏定义  默认 P12_1
//      MISO                查看 zf_device_ips200pro.h 中 IPS200PRO_MISO_PIN 宏定义  默认 NULL
//      RST                 查看 zf_device_ips200pro.h 中 IPS200PRO_RST_PIN  宏定义  默认 P22_4
//      INT                 查看 zf_device_ips200pro.h 中 IPS200PRO_INT_PIN  宏定义  默认 P22_3
//      CS                  查看 zf_device_ips200pro.h 中 IPS200PRO_CS_PIN   宏定义  默认 P12_3
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//
// 接入总钻风灰度数字摄像头 对应主板摄像头接口 请注意线序
//      模块管脚            单片机管脚
//      SCL                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX 宏定义 默认 P17_1 总钻风 SCL 引脚 要接在单片机对应的 GPIO 上
//      SDA                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX 宏定义 默认 P17_2 总钻风 SDA 引脚 要接在单片机对应的 GPIO 上
//      D0                  查看 zf_device_mt9v03x.h 中 MT9V03X_D0_PIN      宏定义 默认 P18_0
//      D1                  查看 zf_device_mt9v03x.h 中 MT9V03X_D1_PIN      宏定义 默认 P18_1
//      D2                  查看 zf_device_mt9v03x.h 中 MT9V03X_D2_PIN      宏定义 默认 P18_2
//      D3                  查看 zf_device_mt9v03x.h 中 MT9V03X_D3_PIN      宏定义 默认 P18_3
//      D4                  查看 zf_device_mt9v03x.h 中 MT9V03X_D4_PIN      宏定义 默认 P18_4
//      D5                  查看 zf_device_mt9v03x.h 中 MT9V03X_D5_PIN      宏定义 默认 P18_5
//      D6                  查看 zf_device_mt9v03x.h 中 MT9V03X_D6_PIN      宏定义 默认 P18_6
//      D7                  查看 zf_device_mt9v03x.h 中 MT9V03X_D7_PIN      宏定义 默认 P18_7
//      PCLK                查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN    宏定义 默认 P06_5
//      VSYNC               查看 zf_device_mt9v03x.h 中 MT9V03X_VSY_PIN     宏定义 默认 P06_6

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 主板电池供电
//
// 2，接上屏幕，按一下复位，屏幕上会显示亮起并显示页面
//
// 3.通过按键控制页面

// 短按按键1，如果在页面2，则控制参数名称或者参数的加

// 短按按键2，如果在页面1，则随机显示角度，图像等参数
// 短按按键2，如果在页面2，则控制参数名称或者参数的加

// 短按按键3，如果在页面2，则控制参数名称或者参数的减

// 短按按键4切换页面1和页面2
// 长按按键4切换页面3
//

// **************************** 代码区域 ****************************

uint32 rand_num = 0;
int angle, far_value, image_err, run;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
    // ips200pro初始化，如果flash的数据全是-0.0000，先看看是否完成flash数据初始化，查看ipspro_page_ctrl_flash_init函数即可
    ipspro_page_ctrl_init();

    // 设置页面3的识别结果
    ipspro_page_ctrl_show_page3(1, "显示器");
    ipspro_page_ctrl_show_page3(2, "23");
    ipspro_page_ctrl_show_page3(3, "手机");
    ipspro_page_ctrl_show_page3(4, "万用表");
    ipspro_page_ctrl_show_page3(5, "卷尺");
    ipspro_page_ctrl_show_page3(6, "76");
    ipspro_page_ctrl_show_page3(7, "音响");
    ipspro_page_ctrl_show_page3(8, "电烙铁");
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        // 累计数据充当随机数
        
        rand_num++;
        rand_num %= 10000;

        // 显示页面1的数据
        ipspro_page_ctrl_show_page1(angle, far_value, image_err, run);

        // 页面的按键控制，需要在pit的通道一中断中添加按键回调函数
        ipspro_page_ctrl_key_loop();
      
      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕型号是否与例程所使用的型号对应
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
