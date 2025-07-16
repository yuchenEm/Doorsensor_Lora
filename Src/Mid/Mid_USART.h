#ifndef __MID_USART_H
#define __MID_USART_H

#include "mid_sx1278.h"

typedef enum
{
    USART_IPR_4800 = 1,
    USART_IPR_9600,    
    USART_IPR_14400,    
    USART_IPR_19200,
    USART_IPR_28800,    
    USART_IPR_38400,     
    USART_IPR_57600,
    USART_IPR_115200, 
    USART_IPR_MAX = 8,
}en_usart_ipr;

typedef struct
{
    en_commode com_Mode;        // communication mode: TT/AT
    en_workmode workmode;
    en_usart_ipr usart_ipr;     // baud rate
    Str_LoraVari loraPara;      // lora parameter
}stru_SystemVari;

extern stru_SystemVari SystemVari;


void mid_usart_init(void);
void mid_usart_Pro(void);

#endif
