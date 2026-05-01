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
* 文件名称          gps_service
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-01-25       user               V2.0 - 深度重构版本
*                                     - 流式状态机解析器
*                                     - 局部坐标系转换
*                                     - HDOP信号质量评估
********************************************************************************************************************/

//====================================================================================================================
// 严禁在此模块中引用摄像头相关的任何头文件（解耦要求）
//====================================================================================================================

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "zf_driver_uart.h"
#include "zf_driver_delay.h"
#include "zf_driver_timer.h"

#include "gps_service.h"


//==================================================== 全局变量定义 ====================================================
gps_data_struct     gps_data;                               // GPS数据结构体（外部可访问）
gps_origin_struct   gps_origin;                             // GPS原点结构体（外部可访问）
//==================================================== 全局变量定义 ====================================================


//==================================================== 内部变量定义 ====================================================
static uint8        gps_initialized = 0;                    // 初始化完成标志

//-------------------- 流式状态机解析器变量 --------------------
static gps_parse_state_enum     parse_state = GPS_PARSE_IDLE;   // 当前解析状态
static uint8        line_buffer[GPS_LINE_BUFFER_SIZE];      // 当前正在接收的语句缓冲区
static uint8        line_index = 0;                         // 缓冲区写入索引
static uint8        checksum_calc = 0;                      // 计算得到的校验和
static uint8        checksum_recv = 0;                      // 接收到的校验和
static uint8        checksum_char_count = 0;                // 校验和字符计数

//-------------------- 完整语句队列（环形缓冲） --------------------
static uint8        sentence_queue[GPS_SENTENCE_QUEUE_SIZE][GPS_LINE_BUFFER_SIZE];
static uint8        sentence_len[GPS_SENTENCE_QUEUE_SIZE];  // 每条语句的长度
static vuint8       queue_head = 0;                         // 队列头指针（写入位置）
static vuint8       queue_tail = 0;                         // 队列尾指针（读取位置）
static vuint8       queue_count = 0;                        // 队列中的语句数量
//==================================================== 内部变量定义 ====================================================


//==================================================== TAU1201配置命令 ====================================================
// 设置更新速率为10Hz
static const uint8 CMD_SET_RATE_10HZ[] = {
    0xF1, 0xD9, 0x06, 0x42, 0x14, 0x00, 0x00, 0x0A, 0x05, 0x00, 
    0x64, 0x00, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0xD0, 0x07, 
    0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xB8, 0xED
};

// 开启GGA语句输出
static const uint8 CMD_OPEN_GGA[] = {
    0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10
};

// 开启RMC语句输出
static const uint8 CMD_OPEN_RMC[] = {
    0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x01, 0x00, 0x1A
};

// 关闭不需要的语句
static const uint8 CMD_CLOSE_GLL[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
static const uint8 CMD_CLOSE_GSA[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
static const uint8 CMD_CLOSE_GSV[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
static const uint8 CMD_CLOSE_VTG[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B};
static const uint8 CMD_CLOSE_ZDA[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D};
static const uint8 CMD_CLOSE_GST[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F};
static const uint8 CMD_CLOSE_TXT[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x40, 0x00, 0x3A, 0x8F};
//==================================================== TAU1201配置命令 ====================================================


//==================================================== 内部函数声明 ====================================================
static uint8                gps_hex_char_to_value(char c);
static uint8                gps_get_field_index(uint8 field_num, const char *str, uint8 max_len);
static double               gps_str_to_double(const char *str);
static int32                gps_str_to_int(const char *str);
static gps_sentence_type_enum   gps_identify_sentence(const char *sentence);
static void                 gps_parse_rmc(const char *sentence);
static void                 gps_parse_gga(const char *sentence);
static void                 gps_utc_to_beijing(gps_svc_time_struct *time);
static void                 gps_evaluate_quality(void);
static void                 gps_reset_parser(void);
//==================================================== 内部函数声明 ====================================================


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS服务模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     gps_service_init();
// 备注信息     初始化UART串口、配置中断接收、配置TAU1201模块参数、初始化状态机
//-------------------------------------------------------------------------------------------------------------------
void gps_service_init(void)
{
    // 初始化GPS数据结构体
    memset(&gps_data, 0, sizeof(gps_data_struct));
    gps_data.status = GPS_STATUS_INVALID;
    gps_data.lat_hemisphere = 'N';
    gps_data.lon_hemisphere = 'E';
    gps_data.hdop = 99.0f;                                  // 初始HDOP设为最大值
    gps_data.quality = GPS_QUALITY_UNKNOWN;
    gps_data.is_reliable = 0;
    gps_data.origin_set = 0;
    
    // 初始化原点结构体
    memset(&gps_origin, 0, sizeof(gps_origin_struct));
    gps_origin.is_set = 0;
    
    // 初始化状态机
    gps_reset_parser();
    
    // 清空语句队列
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    memset(sentence_queue, 0, sizeof(sentence_queue));
    memset(sentence_len, 0, sizeof(sentence_len));
    
    // 等待GPS模块启动
    system_delay_ms(500);
    
    // 初始化UART串口（115200波特率）
    uart_init(GPS_UART, GPS_UART_BAUD, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    
    // 发送TAU1201配置命令
    // 设置更新速率为10Hz
    uart_write_buffer(GPS_UART, (uint8 *)CMD_SET_RATE_10HZ, sizeof(CMD_SET_RATE_10HZ));
    system_delay_ms(100);
    
    // 开启RMC和GGA语句
    uart_write_buffer(GPS_UART, (uint8 *)CMD_OPEN_RMC, sizeof(CMD_OPEN_RMC));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_OPEN_GGA, sizeof(CMD_OPEN_GGA));
    system_delay_ms(50);
    
    // 关闭不需要的语句（减少数据量，提高解析效率）
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_GLL, sizeof(CMD_CLOSE_GLL));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_GSA, sizeof(CMD_CLOSE_GSA));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_GSV, sizeof(CMD_CLOSE_GSV));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_VTG, sizeof(CMD_CLOSE_VTG));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_ZDA, sizeof(CMD_CLOSE_ZDA));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_GST, sizeof(CMD_CLOSE_GST));
    system_delay_ms(50);
    uart_write_buffer(GPS_UART, (uint8 *)CMD_CLOSE_TXT, sizeof(CMD_CLOSE_TXT));
    system_delay_ms(50);
    
    // 使能UART接收中断
    uart_rx_interrupt(GPS_UART, 1);
    
    // 标记初始化完成
    gps_initialized = 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置解析器状态机
// 参数说明     void
// 返回参数     void
// 使用示例     gps_reset_parser();
// 备注信息     内部使用，在接收错误或完成时重置状态
//-------------------------------------------------------------------------------------------------------------------
static void gps_reset_parser(void)
{
    parse_state = GPS_PARSE_IDLE;
    line_index = 0;
    checksum_calc = 0;
    checksum_recv = 0;
    checksum_char_count = 0;
    memset(line_buffer, 0, GPS_LINE_BUFFER_SIZE);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS串口接收中断回调（流式状态机版本）
// 参数说明     void
// 返回参数     void
// 使用示例     在 uart2_isr() 中调用 gps_uart_callback();
// 备注信息     【核心改进】逐字符读入，使用状态机识别完整语句
//             状态转换：IDLE -> HEADER -> BODY -> CHECKSUM1 -> CHECKSUM2 -> CR -> LF -> COMPLETE
//             优势：不会丢失连续报文，支持10Hz高频数据流
//-------------------------------------------------------------------------------------------------------------------
void gps_uart_callback(void)
{
    uint8 dat;
    
    if(!gps_initialized)
    {
        // 未初始化时清空接收缓冲
        while(uart_query_byte(GPS_UART, &dat));
        return;
    }
    
    // 逐字符处理
    while(uart_query_byte(GPS_UART, &dat))
    {
        switch(parse_state)
        {
            //-------------------- 空闲状态：等待'$'开始符 --------------------
            case GPS_PARSE_IDLE:
                if('$' == dat)
                {
                    // 检测到语句开始
                    line_index = 0;
                    checksum_calc = 0;
                    line_buffer[line_index++] = dat;
                    parse_state = GPS_PARSE_HEADER;
                }
                // 其他字符忽略
                break;
                
            //-------------------- 接收语句头（$后的5-6个字符） --------------------
            case GPS_PARSE_HEADER:
                if(line_index < GPS_LINE_BUFFER_SIZE - 1)
                {
                    line_buffer[line_index++] = dat;
                    checksum_calc ^= dat;                   // 校验和计算（从$后开始）
                    
                    // 语句头通常为6字符（如$GNRMC），等接收足够长度后进入BODY状态
                    if(line_index >= 6)
                    {
                        parse_state = GPS_PARSE_BODY;
                    }
                }
                else
                {
                    // 缓冲区溢出，重置
                    gps_reset_parser();
                }
                
                // 检测异常情况（语句头中不应出现这些字符）
                if('*' == dat || '\r' == dat || '\n' == dat || '$' == dat)
                {
                    gps_reset_parser();
                    if('$' == dat)
                    {
                        // 新语句开始
                        line_buffer[line_index++] = dat;
                        parse_state = GPS_PARSE_HEADER;
                    }
                }
                break;
                
            //-------------------- 接收语句体（逗号分隔的数据字段） --------------------
            case GPS_PARSE_BODY:
                if('*' == dat)
                {
                    // 检测到校验和分隔符
                    line_buffer[line_index++] = dat;
                    parse_state = GPS_PARSE_CHECKSUM1;
                    checksum_char_count = 0;
                }
                else if('$' == dat)
                {
                    // 意外的新语句开始，放弃当前语句
                    gps_reset_parser();
                    line_buffer[line_index++] = dat;
                    parse_state = GPS_PARSE_HEADER;
                }
                else if('\r' == dat || '\n' == dat)
                {
                    // 意外的行结束，放弃当前语句
                    gps_reset_parser();
                }
                else if(line_index < GPS_LINE_BUFFER_SIZE - 1)
                {
                    line_buffer[line_index++] = dat;
                    checksum_calc ^= dat;                   // 继续累加校验和
                }
                else
                {
                    // 缓冲区溢出，重置
                    gps_reset_parser();
                }
                break;
                
            //-------------------- 接收校验和高位（*后第1个字符） --------------------
            case GPS_PARSE_CHECKSUM1:
                line_buffer[line_index++] = dat;
                checksum_recv = gps_hex_char_to_value(dat) << 4;
                parse_state = GPS_PARSE_CHECKSUM2;
                break;
                
            //-------------------- 接收校验和低位（*后第2个字符） --------------------
            case GPS_PARSE_CHECKSUM2:
                line_buffer[line_index++] = dat;
                checksum_recv |= gps_hex_char_to_value(dat);
                parse_state = GPS_PARSE_CR;
                break;
                
            //-------------------- 等待回车符 \r --------------------
            case GPS_PARSE_CR:
                if('\r' == dat)
                {
                    parse_state = GPS_PARSE_LF;
                }
                else
                {
                    // 格式错误，重置
                    gps_reset_parser();
                }
                break;
                
            //-------------------- 等待换行符 \n --------------------
            case GPS_PARSE_LF:
                if('\n' == dat)
                {
                    // 语句接收完成
                    line_buffer[line_index] = '\0';         // 添加字符串结束符
                    
                    // 校验和验证
                    if(checksum_calc == checksum_recv)
                    {
                        // 校验通过，将语句放入队列
                        if(queue_count < GPS_SENTENCE_QUEUE_SIZE)
                        {
                            memcpy(sentence_queue[queue_head], line_buffer, line_index + 1);
                            sentence_len[queue_head] = line_index;
                            queue_head = (queue_head + 1) % GPS_SENTENCE_QUEUE_SIZE;
                            queue_count++;
                        }
                        // 队列满时丢弃最旧的语句（覆盖写入）
                        else
                        {
                            memcpy(sentence_queue[queue_head], line_buffer, line_index + 1);
                            sentence_len[queue_head] = line_index;
                            queue_head = (queue_head + 1) % GPS_SENTENCE_QUEUE_SIZE;
                            queue_tail = (queue_tail + 1) % GPS_SENTENCE_QUEUE_SIZE;
                        }
                    }
                    else
                    {
                        // 校验和错误计数
                        gps_data.checksum_error_count++;
                    }
                }
                // 无论成功失败，重置状态机准备接收下一条
                gps_reset_parser();
                break;
                
            default:
                gps_reset_parser();
                break;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS数据更新（主循环调用）
// 参数说明     void
// 返回参数     uint8           0-无新数据 1-有新数据已更新
// 使用示例     if(gps_service_update()) { /* 处理新数据 */ }
// 备注信息     从语句队列中取出完整语句并解析，更新gps_data结构体
//-------------------------------------------------------------------------------------------------------------------
uint8 gps_service_update(void)
{
    uint8 updated = 0;
    gps_sentence_type_enum sentence_type;
    char *sentence;
    
    if(!gps_initialized)
    {
        return 0;
    }
    
    // 处理队列中的所有语句
    while(queue_count > 0)
    {
        // 取出队列尾部的语句
        sentence = (char *)sentence_queue[queue_tail];
        
        // 识别语句类型
        sentence_type = gps_identify_sentence(sentence);
        
        // 根据类型解析
        switch(sentence_type)
        {
            case GPS_SENTENCE_RMC:
                gps_parse_rmc(sentence);
                gps_data.rmc_count++;
                updated = 1;
                break;
                
            case GPS_SENTENCE_GGA:
                gps_parse_gga(sentence);
                gps_data.gga_count++;
                updated = 1;
                break;
                
            default:
                // 未知语句，忽略
                break;
        }
        
        // 移动队列尾指针
        queue_tail = (queue_tail + 1) % GPS_SENTENCE_QUEUE_SIZE;
        queue_count--;
    }
    
    // 如果有更新，执行后处理
    if(updated)
    {
        // 评估信号质量
        gps_evaluate_quality();
        
        // 如果已设置原点且定位有效，更新局部坐标
        if(gps_origin.is_set && gps_data.status != GPS_STATUS_INVALID)
        {
            gps_update_local_xy();
        }
        
        // 更新标志和时间戳
        gps_data.is_updated = 1;
        // Note: timestamp disabled (system_getval_ms not available)
        // gps_data.update_timestamp = system_getval_ms();
    }
    
    return updated;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     十六进制字符转换为数值
// 参数说明     c               十六进制字符（'0'-'9', 'A'-'F', 'a'-'f'）
// 返回参数     uint8           对应的数值（0-15）
// 使用示例     value = gps_hex_char_to_value('A');  // 返回10
// 备注信息     内部使用，用于校验和解析
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_hex_char_to_value(char c)
{
    if(c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else if(c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    else if(c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     识别NMEA语句类型
// 参数说明     *sentence       NMEA语句字符串
// 返回参数     gps_sentence_type_enum  语句类型
// 使用示例     type = gps_identify_sentence("$GNRMC,...");
// 备注信息     内部使用，支持$GNRMC/$GPRMC和$GNGGA/$GPGGA
//-------------------------------------------------------------------------------------------------------------------
static gps_sentence_type_enum gps_identify_sentence(const char *sentence)
{
    // 检查语句头（跳过$符号）
    if(sentence[0] != '$')
    {
        return GPS_SENTENCE_UNKNOWN;
    }
    
    // 检查RMC语句（$GNRMC 或 $GPRMC）
    if((strncmp(&sentence[3], "RMC", 3) == 0) || (strncmp(&sentence[2], "RMC", 3) == 0))
    {
        return GPS_SENTENCE_RMC;
    }
    
    // 检查GGA语句（$GNGGA 或 $GPGGA）
    if((strncmp(&sentence[3], "GGA", 3) == 0) || (strncmp(&sentence[2], "GGA", 3) == 0))
    {
        return GPS_SENTENCE_GGA;
    }
    
    return GPS_SENTENCE_UNKNOWN;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取NMEA语句中指定字段的起始索引
// 参数说明     field_num       字段编号（从1开始，1表示第一个','后的字段）
// 参数说明     *str            NMEA语句字符串
// 参数说明     max_len         最大搜索长度
// 返回参数     uint8           字段起始索引，0表示未找到
// 使用示例     index = gps_get_field_index(3, sentence, 100);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_get_field_index(uint8 field_num, const char *str, uint8 max_len)
{
    uint8 i;
    uint8 comma_count = 0;
    
    for(i = 0; i < max_len && str[i] != '\0'; i++)
    {
        if(',' == str[i])
        {
            comma_count++;
            if(comma_count == field_num)
            {
                return i + 1;  // 返回逗号后一个位置
            }
        }
    }
    
    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将字符串转换为双精度浮点数（到下一个逗号为止）
// 参数说明     *str            字符串指针
// 返回参数     double          转换后的浮点值
// 使用示例     value = gps_str_to_double(&sentence[index]);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static double gps_str_to_double(const char *str)
{
    char buf[20] = {0};
    uint8 i = 0;
    
    while(str[i] != ',' && str[i] != '*' && str[i] != '\0' && i < 19)
    {
        buf[i] = str[i];
        i++;
    }
    buf[i] = '\0';
    
    return atof(buf);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将字符串转换为整数（到下一个逗号为止）
// 参数说明     *str            字符串指针
// 返回参数     int32           转换后的整数值
// 使用示例     value = gps_str_to_int(&sentence[index]);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static int32 gps_str_to_int(const char *str)
{
    char buf[16] = {0};
    uint8 i = 0;
    
    while(str[i] != ',' && str[i] != '*' && str[i] != '\0' && i < 15)
    {
        buf[i] = str[i];
        i++;
    }
    buf[i] = '\0';
    
    return atoi(buf);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析RMC语句（推荐最小定位信息）
// 参数说明     *sentence       RMC语句字符串
// 返回参数     void
// 使用示例     gps_parse_rmc(sentence);
// 备注信息     RMC格式：$GNRMC,hhmmss.ss,A,ddmm.mmmmm,N,dddmm.mmmmm,E,speed,course,ddmmyy,,,A*xx
//-------------------------------------------------------------------------------------------------------------------
static void gps_parse_rmc(const char *sentence)
{
    uint8 idx;
    char status;
    double lat_raw, lon_raw;
    double lat_min, lon_min;
    float speed_knots;
    
    // 字段2：定位状态 A=有效 V=无效 D=差分
    idx = gps_get_field_index(2, sentence, GPS_LINE_BUFFER_SIZE);
    if(idx == 0) return;
    status = sentence[idx];
    
    if('A' == status || 'D' == status)
    {
        gps_data.status = ('D' == status) ? GPS_STATUS_DGPS : GPS_STATUS_VALID;
        
        // 字段3：纬度 ddmm.mmmmm
        idx = gps_get_field_index(3, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx == 0 || sentence[idx] == ',') return;
        lat_raw = gps_str_to_double(&sentence[idx]);
        
        // 字段4：纬度半球 N/S
        idx = gps_get_field_index(4, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx == 0) return;
        gps_data.lat_hemisphere = sentence[idx];
        
        // 字段5：经度 dddmm.mmmmm
        idx = gps_get_field_index(5, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx == 0 || sentence[idx] == ',') return;
        lon_raw = gps_str_to_double(&sentence[idx]);
        
        // 字段6：经度半球 E/W
        idx = gps_get_field_index(6, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx == 0) return;
        gps_data.lon_hemisphere = sentence[idx];
        
        // 转换纬度为十进制度数
        gps_data.lat_degree = (uint16)(lat_raw / 100);
        lat_min = lat_raw - gps_data.lat_degree * 100;
        gps_data.lat_minute = (uint16)lat_min;
        gps_data.lat_second = (uint16)((lat_min - gps_data.lat_minute) * 6000);
        gps_data.latitude = (double)gps_data.lat_degree + lat_min / 60.0;
        if('S' == gps_data.lat_hemisphere)
        {
            gps_data.latitude = -gps_data.latitude;
        }
        
        // 转换经度为十进制度数
        gps_data.lon_degree = (uint16)(lon_raw / 100);
        lon_min = lon_raw - gps_data.lon_degree * 100;
        gps_data.lon_minute = (uint16)lon_min;
        gps_data.lon_second = (uint16)((lon_min - gps_data.lon_minute) * 6000);
        gps_data.longitude = (double)gps_data.lon_degree + lon_min / 60.0;
        if('W' == gps_data.lon_hemisphere)
        {
            gps_data.longitude = -gps_data.longitude;
        }
        
        // 字段7：地面速率（节）-> 转换为km/h
        idx = gps_get_field_index(7, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx > 0 && sentence[idx] != ',')
        {
            speed_knots = (float)gps_str_to_double(&sentence[idx]);
            gps_data.speed = speed_knots * 1.852f;          // 1节 = 1.852 km/h
        }
        
        // 字段8：地面航向（度）
        idx = gps_get_field_index(8, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx > 0 && sentence[idx] != ',')
        {
            gps_data.course = (float)gps_str_to_double(&sentence[idx]);
        }
    }
    else
    {
        gps_data.status = GPS_STATUS_INVALID;
    }
    
    // 字段1：UTC时间 hhmmss.ss（定位有效与否都解析）
    idx = gps_get_field_index(1, sentence, GPS_LINE_BUFFER_SIZE);
    if(idx > 0 && sentence[idx] != ',')
    {
        gps_data.time.hour   = (sentence[idx + 0] - '0') * 10 + (sentence[idx + 1] - '0');
        gps_data.time.minute = (sentence[idx + 2] - '0') * 10 + (sentence[idx + 3] - '0');
        gps_data.time.second = (sentence[idx + 4] - '0') * 10 + (sentence[idx + 5] - '0');
    }
    
    // 字段9：UTC日期 ddmmyy
    idx = gps_get_field_index(9, sentence, GPS_LINE_BUFFER_SIZE);
    if(idx > 0 && sentence[idx] != ',')
    {
        gps_data.time.day   = (sentence[idx + 0] - '0') * 10 + (sentence[idx + 1] - '0');
        gps_data.time.month = (sentence[idx + 2] - '0') * 10 + (sentence[idx + 3] - '0');
        gps_data.time.year  = (sentence[idx + 4] - '0') * 10 + (sentence[idx + 5] - '0') + 2000;
    }
    
    // UTC时间转换为北京时间（UTC+8）
    gps_utc_to_beijing(&gps_data.time);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析GGA语句（GPS定位信息，包含HDOP）
// 参数说明     *sentence       GGA语句字符串
// 返回参数     void
// 使用示例     gps_parse_gga(sentence);
// 备注信息     GGA格式：$GNGGA,hhmmss.ss,lat,N,lon,E,quality,satnum,hdop,alt,M,undulation,M,age,stn*xx
//-------------------------------------------------------------------------------------------------------------------
static void gps_parse_gga(const char *sentence)
{
    uint8 idx;
    char quality;
    
    // 字段6：定位质量 0=无效 1=GPS定位 2=DGPS定位
    idx = gps_get_field_index(6, sentence, GPS_LINE_BUFFER_SIZE);
    if(idx == 0) return;
    quality = sentence[idx];
    
    if(quality != '0' && quality != ',')
    {
        // 字段7：用于定位的卫星数
        idx = gps_get_field_index(7, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx > 0 && sentence[idx] != ',')
        {
            gps_data.satellite_count = (uint8)gps_str_to_int(&sentence[idx]);
        }
        
        // 字段8：HDOP（水平精度因子）【新增】
        idx = gps_get_field_index(8, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx > 0 && sentence[idx] != ',')
        {
            gps_data.hdop = (float)gps_str_to_double(&sentence[idx]);
        }
        
        // 字段9：海拔高度（米）
        idx = gps_get_field_index(9, sentence, GPS_LINE_BUFFER_SIZE);
        if(idx > 0 && sentence[idx] != ',')
        {
            float alt = (float)gps_str_to_double(&sentence[idx]);
            
            // 字段11：大地水准面高度（需要加上）
            idx = gps_get_field_index(11, sentence, GPS_LINE_BUFFER_SIZE);
            if(idx > 0 && sentence[idx] != ',')
            {
                alt += (float)gps_str_to_double(&sentence[idx]);
            }
            
            gps_data.altitude = alt;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UTC时间转换为北京时间（UTC+8）
// 参数说明     *time           时间结构体指针
// 返回参数     void
// 使用示例     gps_utc_to_beijing(&gps_data.time);
// 备注信息     北京时间 = UTC + 8小时，需要处理跨日、跨月、跨年
//-------------------------------------------------------------------------------------------------------------------
static void gps_utc_to_beijing(gps_svc_time_struct *time)
{
    uint8 days_in_month;
    
    // 加8小时
    time->hour += 8;
    
    // 如果超过24小时，需要进位到下一天
    if(time->hour >= 24)
    {
        time->hour -= 24;
        time->day += 1;
        
        // 计算当月天数
        if(2 == time->month)
        {
            // 2月：判断闰年
            days_in_month = 28;
            if((0 == time->year % 4 && 0 != time->year % 100) || 0 == time->year % 400)
            {
                days_in_month = 29;                         // 闰年2月29天
            }
        }
        else if(4 == time->month || 6 == time->month || 9 == time->month || 11 == time->month)
        {
            days_in_month = 30;                             // 4、6、9、11月30天
        }
        else
        {
            days_in_month = 31;                             // 其他月31天
        }
        
        // 如果超过当月天数，进位到下一月
        if(time->day > days_in_month)
        {
            time->day = 1;
            time->month += 1;
            
            // 如果超过12月，进位到下一年
            if(time->month > 12)
            {
                time->month = 1;
                time->year += 1;
            }
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     评估GPS信号质量
// 参数说明     void
// 返回参数     void
// 使用示例     gps_evaluate_quality();
// 备注信息     根据HDOP值评估信号质量，HDOP >= 4.0 标记为不可靠
//-------------------------------------------------------------------------------------------------------------------
static void gps_evaluate_quality(void)
{
    // 根据HDOP评估质量等级
    if(gps_data.hdop < GPS_HDOP_EXCELLENT)
    {
        gps_data.quality = GPS_QUALITY_EXCELLENT;
        gps_data.is_reliable = 1;
    }
    else if(gps_data.hdop < GPS_HDOP_GOOD)
    {
        gps_data.quality = GPS_QUALITY_GOOD;
        gps_data.is_reliable = 1;
    }
    else if(gps_data.hdop < GPS_HDOP_MODERATE)
    {
        gps_data.quality = GPS_QUALITY_MODERATE;
        gps_data.is_reliable = 1;
    }
    else
    {
        gps_data.quality = GPS_QUALITY_UNRELIABLE;
        gps_data.is_reliable = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置GPS原点（将当前位置设为赛道起点）
// 参数说明     void
// 返回参数     uint8           0-设置失败（未定位） 1-设置成功
// 使用示例     if(gps_set_origin()) { printf("Origin set!\n"); }
// 备注信息     记录当前经纬度为局部坐标系原点，预计算cos(lat)提高后续转换效率
//-------------------------------------------------------------------------------------------------------------------
uint8 gps_set_origin(void)
{
    // 检查是否已定位
    if(gps_data.status == GPS_STATUS_INVALID)
    {
        return 0;
    }
    
    // 记录原点坐标
    gps_origin.latitude = gps_data.latitude;
    gps_origin.longitude = gps_data.longitude;
    
    // 预计算cos(lat)，使用硬浮点单元加速
    // CYT4BB7 Cortex-M7 内置 FPU，float运算会自动使用
    gps_origin.cos_lat = (float)cos(gps_origin.latitude * GPS_DEG_TO_RAD);
    
    // 标记原点已设置
    gps_origin.is_set = 1;
    gps_data.origin_set = 1;
    
    // 重置局部坐标
    gps_data.pos_x = 0.0f;
    gps_data.pos_y = 0.0f;
    
    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置指定坐标为GPS原点
// 参数说明     lat             原点纬度（十进制度数）
// 参数说明     lon             原点经度（十进制度数）
// 返回参数     void
// 使用示例     gps_set_origin_manual(30.123456, 104.654321);
// 备注信息     手动指定原点坐标，用于已知起点的情况
//-------------------------------------------------------------------------------------------------------------------
void gps_set_origin_manual(double lat, double lon)
{
    // 记录原点坐标
    gps_origin.latitude = lat;
    gps_origin.longitude = lon;
    
    // 预计算cos(lat)
    gps_origin.cos_lat = (float)cos(lat * GPS_DEG_TO_RAD);
    
    // 标记原点已设置
    gps_origin.is_set = 1;
    gps_data.origin_set = 1;
    
    // 重置局部坐标
    gps_data.pos_x = 0.0f;
    gps_data.pos_y = 0.0f;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新局部坐标（LLH转Local XY）
// 参数说明     void
// 返回参数     void
// 使用示例     gps_update_local_xy();
// 备注信息     【核心算法】使用等距圆柱投影（Equirectangular projection）
//             公式：x = (lon - lon0) * 111320 * cos(lat0)  （东向为正）
//                   y = (lat - lat0) * 111320              （北向为正）
//             精度：在赛道尺度（<10km）下误差<1%，满足智能车需求
//             性能：使用预计算的cos_lat，避免重复三角函数运算
//-------------------------------------------------------------------------------------------------------------------
void gps_update_local_xy(void)
{
    double delta_lon, delta_lat;
    
    // 检查原点是否已设置
    if(!gps_origin.is_set)
    {
        return;
    }
    
    // 计算经纬度差值（度）
    delta_lon = gps_data.longitude - gps_origin.longitude;
    delta_lat = gps_data.latitude - gps_origin.latitude;
    
    // 等距圆柱投影转换为米
    // X轴（东向）：考虑纬度对经度距离的影响
    // 使用预计算的cos_lat提高效率（CYT4BB7 FPU float运算快）
    gps_data.pos_x = (float)(delta_lon * GPS_DEG_TO_M_LAT * gps_origin.cos_lat);
    
    // Y轴（北向）：纬度1度约等于111320米
    gps_data.pos_y = (float)(delta_lat * GPS_DEG_TO_M_LAT);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两点之间的距离（Haversine公式）
// 参数说明     lat1            第一个点的纬度（十进制度数）
// 参数说明     lon1            第一个点的经度（十进制度数）
// 参数说明     lat2            第二个点的纬度（十进制度数）
// 参数说明     lon2            第二个点的经度（十进制度数）
// 返回参数     double          两点之间的距离（单位：米）
// 使用示例     distance = gps_calc_distance(30.123, 104.456, 30.234, 104.567);
// 备注信息     使用WGS-84椭球体模型，精度约0.5%
//-------------------------------------------------------------------------------------------------------------------
double gps_calc_distance(double lat1, double lon1, double lat2, double lon2)
{
    double rad_lat1, rad_lat2;
    double rad_lon1, rad_lon2;
    double delta_lat, delta_lon;
    double a, c, distance;
    
    // 转换为弧度
    rad_lat1 = lat1 * GPS_DEG_TO_RAD;
    rad_lat2 = lat2 * GPS_DEG_TO_RAD;
    rad_lon1 = lon1 * GPS_DEG_TO_RAD;
    rad_lon2 = lon2 * GPS_DEG_TO_RAD;
    
    delta_lat = rad_lat2 - rad_lat1;
    delta_lon = rad_lon2 - rad_lon1;
    
    // Haversine公式
    a = sin(delta_lat / 2) * sin(delta_lat / 2) +
        cos(rad_lat1) * cos(rad_lat2) * sin(delta_lon / 2) * sin(delta_lon / 2);
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    distance = GPS_EARTH_RADIUS_M * c;
    
    return distance;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的方位角
// 参数说明     lat1            第一个点的纬度（十进制度数）
// 参数说明     lon1            第一个点的经度（十进制度数）
// 参数说明     lat2            第二个点的纬度（十进制度数）
// 参数说明     lon2            第二个点的经度（十进制度数）
// 返回参数     double          方位角（0-360度，以真北为参考，顺时针）
// 使用示例     azimuth = gps_calc_azimuth(30.123, 104.456, 30.234, 104.567);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
double gps_calc_azimuth(double lat1, double lon1, double lat2, double lon2)
{
    double rad_lat1, rad_lat2;
    double rad_lon1, rad_lon2;
    double x, y;
    double azimuth;
    
    // 转换为弧度
    rad_lat1 = lat1 * GPS_DEG_TO_RAD;
    rad_lat2 = lat2 * GPS_DEG_TO_RAD;
    rad_lon1 = lon1 * GPS_DEG_TO_RAD;
    rad_lon2 = lon2 * GPS_DEG_TO_RAD;
    
    // 计算方位角
    x = sin(rad_lon2 - rad_lon1) * cos(rad_lat2);
    y = cos(rad_lat1) * sin(rad_lat2) - sin(rad_lat1) * cos(rad_lat2) * cos(rad_lon2 - rad_lon1);
    azimuth = atan2(x, y) * GPS_RAD_TO_DEG;
    
    // 转换为0-360度
    if(azimuth < 0)
    {
        azimuth += 360.0;
    }
    
    return azimuth;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测GPS是否已定位
// 参数说明     void
// 返回参数     uint8           0-未定位 1-已定位
// 使用示例     if(gps_is_fixed()) { /* GPS已锁定卫星 */ }
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 gps_is_fixed(void)
{
    return (gps_data.status != GPS_STATUS_INVALID) ? 1 : 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测GPS数据是否可靠
// 参数说明     void
// 返回参数     uint8           0-不可靠(HDOP>=4.0) 1-可靠
// 使用示例     if(gps_is_reliable()) { /* 数据可信 */ }
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 gps_is_reliable(void)
{
    return gps_data.is_reliable;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取到原点的距离
// 参数说明     void
// 返回参数     float           到原点的距离（米），未设置原点返回-1
// 使用示例     dist = gps_get_distance_to_origin();
// 备注信息     使用局部坐标系计算（sqrt(x^2 + y^2)），效率高于经纬度直接计算
//-------------------------------------------------------------------------------------------------------------------
float gps_get_distance_to_origin(void)
{
    if(!gps_origin.is_set)
    {
        return -1.0f;
    }
    
    // 使用局部坐标计算距离（FPU加速）
    return sqrtf(gps_data.pos_x * gps_data.pos_x + gps_data.pos_y * gps_data.pos_y);
}
