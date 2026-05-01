#include "TOF.h"
#include "zf_common_headfile.h"

#define LED1                          (P19_0)
 uint16 g_tof_distance_mm = 0;
static uint16 tof_distance_filtered = 0;
static uint8 tof_init_count = 0;
void tof_init(void)
{
    g_tof_distance_mm = 0;
    tof_distance_filtered = 0;
    tof_init_count = 0;
    while(1)
    {
        if(dl1b_init())
        {
          gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
          gpio_set_level(LED1, GPIO_LOW); 
          printf("\r\n tof_init: dl1b init error.");  
           system_delay_ms(30);   
        }
        else
            break;
                                                       
    }
}

void tof_update(void)
{
    dl1b_get_distance();
    if(dl1b_finsh_flag == 1)
    {
        dl1b_finsh_flag = 0;
        if(tof_init_count < 10)
        {
            tof_distance_filtered = dl1b_distance_mm;
            tof_init_count++;
        }
        else
        {
            tof_distance_filtered = (tof_distance_filtered * 7 + dl1b_distance_mm * 3) / 10;
        } 


        g_tof_distance_mm = tof_distance_filtered;
    }
}

