#include "stm8l10x.h"
#include "queue.h"


/********************************************************************************************************
    @Name       : S_QueueEmpty
    @Function   : empty(Initialize) a queue
    @Parameters : Head->head pointer
                  Tail->tail pointer
                  HBuff->queue buffer
********************************************************************************************************/
void S_QueueEmpty(unsigned char **Head, unsigned char **Tail, unsigned char *HBuff)
{
    *Head = HBuff;
    *Tail = HBuff;
}

/********************************************************************************************************
    @Name       : S_QueueDataIn
    @Function   : input data to a queue
    @Parameters : Head->head pointer
                  Tail->tail pointer
                  HBuff->queue buffer
                  Len->queue buffer length
                  HData->data buffer
                  DataLen->data length
********************************************************************************************************/
void S_QueueDataIn(unsigned char **Head, unsigned char **Tail, unsigned char *HBuff, unsigned short Len, unsigned char *HData, unsigned short DataLen)
{
    unsigned short num;
    unsigned char IptStatus;

    IptStatus = __get_interrupt_state();
        
    disableInterrupts();
        
    for(num = 0; num < DataLen; num++, HData++)
    {
        **Tail = *HData;
        (*Tail)++;
        if(*Tail == HBuff+Len)
        *Tail = HBuff;
        if(*Tail == *Head)
        {
            if(++(*Head) == HBuff+Len)
            *Head = HBuff;
        }
    }
    __set_interrupt_state(IptStatus);
}

/********************************************************************************************************
    @Name       : S_QueueDataOut
    @Function   : output data from a queue
    @Parameters : Head->head pointer
                  Tail->tail pointer
                  HBuff->queue buffer
                  Len->queue buffer length
                  Data->data buffer
********************************************************************************************************/
unsigned char S_QueueDataOut(unsigned char **Head, unsigned char **Tail, unsigned char *HBuff, unsigned short Len, unsigned char *Data)
{   
    unsigned char back = 0;
    unsigned char IptStatus;
    IptStatus = __get_interrupt_state();
        
    disableInterrupts();
    *Data = 0;
    if(*Tail != *Head)
    {
        *Data = **Head;
        back = 1; 
        if(++(*Head) == HBuff+Len)
        *Head = HBuff;
    }
    __set_interrupt_state(IptStatus);
    return back;
}

/********************************************************************************************************
    @Name       : S_QueueDataLen
    @Function   : get data length in a queue
    @Parameters : Head->head pointer
                  Tail->tail pointer
                  Len->queue buffer length
********************************************************************************************************/
unsigned short S_QueueDataLen(unsigned char **Head, unsigned char **Tail, unsigned short Len)
{
    if(*Tail > *Head)
    return *Tail-*Head;
    if(*Tail < *Head)
    return *Tail+Len-*Head;
    return 0;
}