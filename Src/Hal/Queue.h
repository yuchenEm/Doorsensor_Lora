#ifndef _QUEUE_H_
#define _QUEUE_H_ 

extern void S_QueueEmpty(unsigned char **Head, unsigned char **Tail, unsigned char *HBuff);
extern void S_QueueDataIn(unsigned char **Head, unsigned char **Tail, unsigned char *HBuff, unsigned short Len, unsigned char *HData, unsigned short DataLen);
extern unsigned char S_QueueDataOut(unsigned char **Head, unsigned char **Tail, unsigned char *HBuff, unsigned short Len, unsigned char *Data);
extern unsigned short S_QueueDataLen(unsigned char **Head, unsigned char **Tail, unsigned short Len);
 
#define QueueEmpty(x)   S_QueueEmpty((unsigned char**)&(x).Head,(unsigned char**)&(x).Tail,(unsigned char*)(x).Buff) 
#define QueueDataIn(x,y,z) S_QueueDataIn((unsigned char**)&(x).Head,(unsigned char**)&(x).Tail,(unsigned char*)(x).Buff,sizeof((x).Buff),(y),(z))
#define QueueDataOut(x,y)  S_QueueDataOut((unsigned char**)&(x).Head,(unsigned char**)&(x).Tail,(unsigned char*)(x).Buff,sizeof((x).Buff),(y)) 
#define QueueDataLen(x)   S_QueueDataLen((unsigned char**)&(x).Head,(unsigned char**)&(x).Tail,sizeof((x).Buff))  

typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[4+1];}		Queue4;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[8+1];}		Queue8;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[16+1];}		Queue16; 
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[32+1];}		Queue32;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[64+1];}   	Queue64;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[90+1];}   	Queue90;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[120+1];}  	Queue120;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[340+1];}  	Queue340;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[745+1];}  	Queue745;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[128+1];}  	Queue128;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[248+1];}  	Queue248;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[256+1];}  	Queue256;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[512+1];}  	Queue512;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[1024+1];} 	Queue1K;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[2048+1];} 	Queue2K;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[5120+1];} 	Queue5K;
typedef struct{unsigned char *Head; unsigned char *Tail; unsigned char Buff[10240+1];} 	Queue10K;

#endif
