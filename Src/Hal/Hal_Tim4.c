/****************************************************
  * @Name   Hal_Tim4.c
  * @Brief  
*****************************************************/

#include "stm8l10x.h"

__IO uint8_t Systick; 

void HAL_TIM4_Config(void){
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  
  TIM4_DeInit();
  
  /* Time base configuration */ 
  TIM4_TimeBaseInit(TIM4_Prescaler_64, 0xFA); //16MHz/64 = 250KHz; 0xFA = 250; TimeBase = 4us
  TIM4_ITConfig(TIM4_IT_Update, ENABLE);
  
  //enableInterrupts();

  /* Initialize I/Os in Output Mode */
  GPIO_Init(GPIOA, GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Fast);

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 25){
    TIM4_ClearITPendingBit(TIM4_IT_Update);
    Systick = 1;
}