/****************************************************
  * @Name   Hal_Sleep.c
  * @Brief  set the Lora module and peripheral to sleep mode
  ***************************************************/

#include "stm8l10x.h"
#include "hal_sleep.h"
#include "mid_usart.h" 

void hal_sleep_init(void)
{   //set all unused GPIO Pin output low
    GPIO_Init(GPIOA, GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|
                        GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, 
                            GPIO_Mode_Out_PP_Low_Slow);
    
    GPIO_Init(GPIOC, GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, 
                        GPIO_Mode_Out_PP_Low_Slow);
    
    GPIO_Init(GPIOD, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|
                        GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, 
                            GPIO_Mode_Out_PP_Low_Slow);  
    
    //GPIO_Init(GPIOA,GPIO_Pin_0,GPIO_Mode_In_PU_IT);  // PA0 EXTI wake up
    EXTI_SetPinSensitivity (EXTI_Pin_0,EXTI_Trigger_Falling);    
}

static void hal_sleep_delay(uint16_t x)
{
    while(x !=0)
    {
        x--;
    }
}

void hal_sleep_task(void)
{
    static unsigned int SleepDelay = 5000; 
    if(SleepDelay)
    {
        if(SystemVari.workmode == WORKMODE_LOW)
        {
            SleepDelay --;
        }
        else
        {
            SleepDelay = 5000;
        }
        
        if(SleepDelay == 0)
        {
            GPIO_Init(GPIOA,GPIO_Pin_0,GPIO_Mode_In_PU_IT); // PA0 EXTI wake up
            mid_Sx1278_Sleep();
            hal_sleep_delay(5);

            halt(); 
            
            hal_sleep_delay(5);
            SleepDelay = 5000;
            mid_SX1278_LoRaRxConfig(); 
            GPIO_Init(GPIOA,GPIO_Pin_0,GPIO_Mode_In_PU_No_IT);
        }          
    }
}

INTERRUPT_HANDLER(EXTI0_IRQHandler, 8)
{
    EXTI_ClearITPendingBit(EXTI_IT_Pin0);
}
