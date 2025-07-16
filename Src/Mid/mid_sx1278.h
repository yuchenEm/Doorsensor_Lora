#ifndef ____MID_SX1278_H_
#define ____MID_SX1278_H_

#define LR_RegFifo                                  0x00 
// Common settings
#define LR_RegOpMode                                0x01
#define LR_RegFrMsb                                 0x06
#define LR_RegFrMid                                 0x07
#define LR_RegFrLsb                                 0x08
// Tx settings
#define LR_RegPaConfig                              0x09
#define LR_RegPaRamp                                0x0A
#define LR_RegOcp                                   0x0B
// Rx settings
#define LR_RegLna                                   0x0C
// LoRa registers
#define LR_RegFifoAddrPtr                           0x0D
#define LR_RegFifoTxBaseAddr                        0x0E
#define LR_RegFifoRxBaseAddr                        0x0F
#define LR_RegFifoRxCurrentaddr                     0x10
#define LR_RegIrqFlagsMask                          0x11
#define LR_RegIrqFlags                              0x12
#define LR_RegRxNbBytes                             0x13
#define LR_RegRxHeaderCntValueMsb                   0x14
#define LR_RegRxHeaderCntValueLsb                   0x15
#define LR_RegRxPacketCntValueMsb                   0x16
#define LR_RegRxPacketCntValueLsb                   0x17
#define LR_RegModemStat                             0x18
#define LR_RegPktSnrValue                           0x19
#define LR_RegPktRssiValue                          0x1A
#define LR_RegRssiValue                             0x1B
#define LR_RegHopChannel                            0x1C
#define LR_RegModemConfig1                          0x1D
#define LR_RegModemConfig2                          0x1E
#define LR_RegSymbTimeoutLsb                        0x1F
#define LR_RegPreambleMsb                           0x20
#define LR_RegPreambleLsb                           0x21
#define LR_RegPayloadLength                         0x22
#define LR_RegMaxPayloadLength                      0x23
#define LR_RegHopPeriod                             0x24
#define LR_RegFifoRxByteAddr                        0x25
#define LR_RegModemConfig3                          0x26

#define LR_RegPacketConfig2                         0x31

#define LR_RegSeqConfig2                            0x37

#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
// Version
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B

#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64

typedef enum
{
    LORAPOWER_2DBM = 2,
    LORAPOWER_3DBM,
    LORAPOWER_4DBM,    
    LORAPOWER_5DBM,
    LORAPOWER_6DBM,
    LORAPOWER_7DBM,
    LORAPOWER_8DBM,    
    LORAPOWER_9DBM,
    LORAPOWER_10DBM,
    LORAPOWER_11DBM,
    LORAPOWER_12DBM,    
    LORAPOWER_13DBM,
    LORAPOWER_14DBM,    
    LORAPOWER_15DBM,
    LORAPOWER_16DBM,
    LORAPOWER_17DBM,
    LORAPOWER_20DBM = 20,    
}en_LoraPower;

typedef enum
{
    LORASF_6 = 6,
    LORASF_7,    
    LORASF_8,    
    LORASF_9,    
    LORASF_10,
    LORASF_11,    
    LORASF_12,           
}en_LoraSf;

typedef enum
{
    LORA_RATE_7_8KHZ,
    LORA_RATE_10_4KHZ,
    LORA_RATE_15_6KHZ,
    LORA_RATE_20_8KHZ,
    LORA_RATE_31_25KHZ,
    LORA_RATE_41_7KHZ,
    LORA_RATE_62_5KHZ,
    LORA_RATE_125KHZ,
    LORA_RATE_250KHZ,
    LORA_RATE_500KHZ,    
}en_LoraRate;

typedef enum
{
    LORA_CR_4_5 = 1,
    LORA_CR_4_6,
    LORA_CR_4_7,
    LORA_CR_4_8,      
}en_LoraCr;

typedef union
{
    unsigned long fredat;
    unsigned char  Fre[4];
}un_lorafre;

typedef struct comdat
{
    en_LoraPower loraPower;
    en_LoraSf loraSf;  
    en_LoraRate loraRate; 
    en_LoraCr loraCr;
    un_lorafre loraFre;
}comdat_typedef;

typedef union
{
    unsigned char buf[64];
    comdat_typedef CommodeVar;
}un_LoraPara;

extern unsigned char lorarxbuf[256];
extern unsigned char lorarxDatLen;  

void mid_SX1278_init(void);


void mid_Sx1278_LoRaSentBuffer(unsigned char *dat,unsigned char len);
void mid_SX1278_LoRaRxConfig(un_LoraPara loraPara);
void mid_Sx1278_RxGpioGonfig(void);
void mid_Sx1278TxGpioConfig(void);
void mid_Sx1278RxDateGet(void);

void mid_Sx1278_Sleep(void);

#endif


