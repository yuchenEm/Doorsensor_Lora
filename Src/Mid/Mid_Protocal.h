#ifndef __MID_PROTOCAL_H_
#define __MID_PROTOcAL_H_

#include "queue.h"

extern volatile Queue64 LoraRxMsg;

/******************************************************
DataFrame:
LORA_COM_APPLY_NET: 19 byte
    Head:           1 byte  0xFE
    Length:         1 byte  0x10
    Data:     16 byte
        Function code:      1 byte  0x01
        CRC16(Node No.):    2 byte  0x0000-0xFFFF
        sensor MAC address: 12 byte
        sensor type:        1 byte
    sumcheck:       1 byte

LORA_COM_Heart - LORA_COM_DOORCLOSE: 6 byte
    Head:               1 byte  0xFE
    Length:             1 byte  0x03
    Data:         3 byte
        Function code:      1 byte  0x02-0x09
        CRC16(Node No.):    2 byte  0x0000-0xFFFF
    sumcheck:           1 byte
*******************************************************/
typedef enum
{
    LORA_COM_APPLY_NET = 1,    // ApplyJoinNet
    LORA_COM_HEART,            // Heartbeat
    LORA_COM_BAT_LOW,          // BatteryLow
    LORA_COM_ALARM,            // Alarm
    
    LORA_COM_DISARM = 6,       // Disarm
    LORA_COM_AWAYARM,          // AwayArm
    LORA_COM_HOMEARM,          // HomeArm
    LORA_COM_DOORCLOSE = 9,    
  
    LORA_COM_RESPOSE_NONODE = 0xFF,  
}en_lora_eventTypedef;

#define LORA_COM_RESPOSE_APPLY_NET  0x81 // sensor join-net reply code

typedef void (*ProtocalEvent_CallBack_t)(unsigned char *data);

void mid_lora_TxDataPackage(en_lora_eventTypedef cmd, unsigned char *pNode);
void mid_Protocal_Init(void);
void mid_Protocal_Pro(void);
void mid_Protocal_CBSRegister(ProtocalEvent_CallBack_t pCBS);

#endif

