/****************************************************
  * @Name   Hal_USART.c
  * @Brief  
*****************************************************/

#include "stm8l10x.h"
#include "hal_usart.h"
#include "queue.h"
#include "mid_sx1278.h"
#include "mid_usart.h"

Queue256 USART_RxQueue;
Queue256 USART_TxQueue;

unsigned char TIM_USART_GetNewDataDelay; //Lora receive data delay
const uint32_t usartIpr_Conf[USART_IPR_MAX] = {4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200};

/*********************************************************************************
typedef struct{
    unsigned char *Head; 
    unsigned char *Tail; 
    unsigned char Buff[4+1];
}Queue4;

void S_QueueEmpty(              #define QueueEmpty(x)   S_QueueEmpty(
    unsigned char **Head,           (unsigned char**)&(x).Head,
    unsigned char **Tail,           (unsigned char**)&(x).Tail,
    unsigned char *HBuff            (unsigned char*)(x).Buff
)                               )

void S_QueueDataIn(             #define QueueDataIn(x,y,z) S_QueueDataIn(
    unsigned char **Head,           (unsigned char**)&(x).Head,
    unsigned char **Tail,           (unsigned char**)&(x).Tail,
    unsigned char *HBuff,           (unsigned char*)(x).Buff,
    unsigned short Len,             sizeof((x).Buff),
    unsigned char *HData,           (y),
    unsigned short DataLen          (z)
)                               )

unsigned char S_QueueDataOut(   #define QueueDataOut(x,y)  S_QueueDataOut(
    unsigned char **Head,           (unsigned char**)&(x).Head,
    unsigned char **Tail,           (unsigned char**)&(x).Tail,
    unsigned char *HBuff,           (unsigned char*)(x).Buff,
    unsigned short Len,             sizeof((x).Buff),
    unsigned char *Data             (y)
)                               )

unsigned short S_QueueDataLen(  #define QueueDataLen(x)   S_QueueDataLen(
    unsigned char **Head,           (unsigned char**)&(x).Head,
    unsigned char **Tail,           (unsigned char**)&(x).Tail,
    unsigned short Len              sizeof((x).Buff)
)                               ) 

**********************************************************************************/

uint8_t TxBuffer[256];      // USART_TX buffer
uint32_t TxCounter = 0;     
uint32_t TxData_Size = 0;   

void HAL_USART_Config(unsigned char rate)
{
    /*High speed internal clock prescaler: 1*/
    //CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv1);
    
    /*Set the USART RX and USART TX at high level*/
    GPIO_ExternalPullUpConfig(GPIOC,GPIO_Pin_3|GPIO_Pin_2, ENABLE);
    
    /* Enable USART clock */
    CLK_PeripheralClockConfig(CLK_Peripheral_USART, ENABLE);
    
    USART_DeInit();
    /* USART configuration ------------------------------------------------------*/
    /* USART configured as follow:
          - BaudRate = 9600 baud  
          - Word Length = 8 Bits
          - One Stop Bit
          - Odd parity
          - Receive and transmit enabled
    */
    USART_Init((uint32_t)usartIpr_Conf[rate - 1], USART_WordLength_8D, USART_StopBits_1,
               USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Rx | USART_Mode_Tx));

    /* Enable the USART Transmit interrupt: this interrupt is generated when the
       USART transmit data register is empty */
    USART_ITConfig(USART_IT_TXE, DISABLE);
    /* Enable the USART Receive interrupt: this interrupt is generated when the
       USART receive data register is not empty */
    USART_ITConfig(USART_IT_RXNE, ENABLE);

    /* Enable general interrupts */
    //enableInterrupts();
    
    QueueEmpty(USART_RxQueue);
    QueueEmpty(USART_TxQueue);
    
    TxCounter = 0;
    TxData_Size = 0;
    TIM_USART_GetNewDataDelay = 0;
}

static void HAL_USART_RxHandler(void)
{
    uint8_t len, i;
    uint8_t USART_RxBuffer[256];
    len = QueueDataLen(USART_RxQueue);
    if(len)
    {
        for(i=0;i<len;i++)
        {
            QueueDataOut(USART_RxQueue, &USART_RxBuffer[i]);
        }
        mid_Sx1278_LoRaSentBuffer(USART_RxBuffer, len);
        //QueueDataIn(USART_TxQueue, USART_RxBuffer, len);
    }
}

static void HAL_USART_TxHandler(void)
{
    uint8_t len, i;
    if(TxCounter == 0){ //TxCounter = 0 indicates IDLE for transmitter
        len = QueueDataLen(USART_TxQueue);
        if(len)
        {
            for(i=0;i<len;i++)
            {
                QueueDataOut(USART_TxQueue, &TxBuffer[i]);
            }
            USART_SendData8(TxBuffer[0]);
            TxCounter = 1;
            TxData_Size = len;
            USART_ITConfig(USART_IT_TXE, ENABLE);
        }
    }
}

void HAL_USART_PutInTxQueue(uint8_t *buff, uint8_t len)
{
    QueueDataIn(USART_TxQueue, buff, len);
}

void HAL_USART_SendStrLen(uint8_t *Str, uint8_t len)
{
    while(len--)
        {
            USART_SendData8(*Str);     
            while(!USART_GetFlagStatus (USART_FLAG_TXE));
            Str++;
        }
}

void HAL_USART_Pro(void)
{
    HAL_USART_TxHandler();
}

INTERRUPT_HANDLER(USART_TX_IRQHandler, 27)
{
    if(TxCounter == TxData_Size)
    {
        TxCounter = 0;
        USART_ITConfig(USART_IT_TXE, DISABLE);
    }
    else
    {
        USART_SendData8(TxBuffer[TxCounter++]);
    }
}

INTERRUPT_HANDLER(USART_RX_IRQHandler, 28)
{
    uint8_t RxData;
    RxData = (uint8_t)USART_ReceiveData8();
    QueueDataIn(USART_RxQueue, &RxData, 1);  
    TIM_USART_GetNewDataDelay = 0;
}