/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* File Name          cm7_0_isr
* IDE                IAR 9.40.1
* MCU                CYT4BB
*
* Changelog
* Date              Author              Note
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            Add PIT timer ISR
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "../code/gps_module/gps_service.h"

extern void gps_debug_callback(void);


// **************************** PIT ISR ****************************
void pit0_ch0_isr()                     // PIT Channel 0 ISR
{
    pit_isr_flag_clear(PIT_CH0);
    // User code here
}

void pit0_ch1_isr()                     // PIT Channel 1 ISR
{
    pit_isr_flag_clear(PIT_CH1);
    
}

void pit0_ch2_isr()                     // PIT Channel 2 ISR
{
    pit_isr_flag_clear(PIT_CH2);
    
}

void pit0_ch10_isr()                    // PIT Channel 10 ISR
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // PIT Channel 11 ISR
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // PIT Channel 12 ISR
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // PIT Channel 13 ISR
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // PIT Channel 14 ISR
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // PIT Channel 15 ISR
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // PIT Channel 16 ISR
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // PIT Channel 17 ISR
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // PIT Channel 18 ISR
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // PIT Channel 19 ISR
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // PIT Channel 20 ISR
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // PIT Channel 21 ISR
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT ISR ****************************


// **************************** GPIO EXTI ISR ****************************
void gpio_0_exti_isr()                  // GPIO_0 EXTI ISR
{
    
  
  
}

void gpio_1_exti_isr()                  // GPIO_1 EXTI ISR
{
    if(exti_flag_get(P01_0))            // P1_0 external interrupt
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // GPIO_2 EXTI ISR
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // GPIO_3 EXTI ISR
{



}

void gpio_4_exti_isr()                  // GPIO_4 EXTI ISR
{



}

void gpio_5_exti_isr()                  // GPIO_5 EXTI ISR
{



}


void gpio_6_exti_isr()                  // GPIO_6 EXTI ISR
{



}

void gpio_7_exti_isr()                  // GPIO_7 EXTI ISR
{



}

void gpio_8_exti_isr()                  // GPIO_8 EXTI ISR
{



}

void gpio_9_exti_isr()                  // GPIO_9 EXTI ISR
{



}

void gpio_10_exti_isr()                 // GPIO_10 EXTI ISR
{



}

void gpio_11_exti_isr()                 // GPIO_11 EXTI ISR
{



}

void gpio_12_exti_isr()                 // GPIO_12 EXTI ISR
{



}

void gpio_13_exti_isr()                 // GPIO_13 EXTI ISR
{



}

void gpio_14_exti_isr()                 // GPIO_14 EXTI ISR
{



}

void gpio_15_exti_isr()                 // GPIO_15 EXTI ISR
{



}

void gpio_16_exti_isr()                 // GPIO_16 EXTI ISR
{



}

void gpio_17_exti_isr()                 // GPIO_17 EXTI ISR
{



}

void gpio_18_exti_isr()                 // GPIO_18 EXTI ISR
{



}

void gpio_19_exti_isr()                 // GPIO_19 EXTI ISR
{



}

void gpio_20_exti_isr()                 // GPIO_20 EXTI ISR
{



}

void gpio_21_exti_isr()                 // GPIO_21 EXTI ISR
{



}

void gpio_22_exti_isr()                 // GPIO_22 EXTI ISR
{



}

void gpio_23_exti_isr()                 // GPIO_23 EXTI ISR
{



}
// **************************** GPIO EXTI ISR ****************************

//// **************************** DMA ISR ****************************
//void dma_event_callback(void* callback_arg, cyhal_dma_event_t event)
//{
//    CY_UNUSED_PARAMETER(event);
//	
//
//	
//	
//}
// **************************** DMA ISR ****************************

// **************************** UART ISR ****************************
// UART0 is default debug UART
void uart0_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_RX_NOT_EMPTY)            // UART0 RX interrupt
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_0), CY_SCB_UART_RX_NOT_EMPTY);              // Clear RX interrupt flag
        
#if DEBUG_UART_USE_INTERRUPT                                                                    // If debug UART interrupt enabled
        debug_interrupr_handler();                                                              // Call debug UART RX handler
#endif                                                                                          // If DEBUG_UART_INDEX changed, move this to corresponding ISR
      
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_TX_DONE)            // UART0 TX interrupt
    {           
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_0), CY_SCB_UART_TX_DONE);                   // Clear TX interrupt flag
        
        
        
    }
}

void uart1_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_RX_NOT_EMPTY)            // UART1 RX interrupt
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_1), CY_SCB_UART_RX_NOT_EMPTY);              // Clear RX interrupt flag

        wireless_uart_callback();                                                               // Wireless UART module callback
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_TX_DONE)            // UART1 TX interrupt
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_1), CY_SCB_UART_TX_DONE);                   // Clear TX interrupt flag
        
        
        
    }
}

void uart2_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_2), CY_SCB_UART_RX_NOT_EMPTY);

        gps_uart_callback();
        gps_debug_callback();
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_2), CY_SCB_UART_TX_DONE);
    }
}

void uart3_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_RX_NOT_EMPTY)            // UART3 RX interrupt
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_3), CY_SCB_UART_RX_NOT_EMPTY);              // Clear RX interrupt flag

        
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_TX_DONE)            // UART3 TX interrupt
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_3), CY_SCB_UART_TX_DONE);                   // Clear TX interrupt flag
        
        
        
    }
}

void uart4_isr (void)
{

    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_RX_NOT_EMPTY)            // UART4 RX interrupt
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_4), CY_SCB_UART_RX_NOT_EMPTY);              // Clear RX interrupt flag




    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_TX_DONE)            // UART4 TX interrupt
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_4), CY_SCB_UART_TX_DONE);                   // Clear TX interrupt flag



    }
}
// **************************** UART ISR ****************************
