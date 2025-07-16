#ifndef __HAL_USART_H_
#define __HAL_USART_H_

#include "queue.h"

extern unsigned char TIM_USART_GetNewDataDelay;

extern Queue256 USART_RxQueue;
extern Queue256 USART_TxQueue;

void HAL_USART_Config(unsigned char rate);
void HAL_USART_Pro(void);
void HAL_USART_PutInTxQueue(uint8_t *buff, uint8_t len);
void HAL_USART_SendStrLen(uint8_t *Str, uint8_t len);

#endif
