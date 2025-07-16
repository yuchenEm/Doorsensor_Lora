#include "mid_sx1278.h"
#include "hal_spi.h"
#include "mid_protocal.h"
#include "queue.h"

#define Sx1278_CRC   0x01 
un_LoraPara loraRegPara;
uint8_t Sx1278_SpreadFat;  

static void mid_sx1278_Config(un_LoraPara loraPara);

__root const uint8_t SX1276SpreadFactorTbl[7] ={6,7,8,9,10,11,12};


unsigned char lorarxbuf[256];
unsigned char lorarxDatLen;  

/*********************************************************************
 * Name         : mid_1278_delay
 * Function     : Millisecond delay function
 * Parameter    : t->xms, Value range: 0-65535
**********************************************************************/
void mid_1278_delay(uint16_t t)
{
  while(t--);
}

/*********************************************************************
 * Name         : mid_sx1278_Rest
 * Function     : SX1278 chip reset operation function
 * Parameter    : None
**********************************************************************/
void mid_sx1278_Rest(void)
{
    RF_RST_LOW;
    mid_1278_delay(300);
    RF_RST_HIGH;
    mid_1278_delay(200);  
}

/*********************************************************************
 * Name         : mid_sx1278_SentPowerConfig
 * Function     : Configure the wireless transmission power of the SX1278 chip registers
 * Parameter    : Power->20dBm, Value range: 1-18
**********************************************************************/
void mid_sx1278_SentPowerConfig(unsigned char Power)
{
  if(Power == 20)
  {
    SX1278WriteReg(REG_LR_PADAC,0x87); 
    SX1278WriteReg(LR_RegPaConfig,0xFF); 
  }
  else if((Power > 1)&&(Power < 18))
  {
    SX1278WriteReg(REG_LR_PADAC,0x84); 
    
    
    SX1278WriteReg(LR_RegPaConfig,(0xf0 + Power - 2)); // 11dbm
  }
}


typedef union 
{
    uint32_t fredat;
    uint8_t  Fre[4];
}un_sx1278Fre;


/*********************************************************************
 * Name         : mid_sx1278_SentFreConfig
 * Function     : Configure the LoRa wireless operating frequency in the SX1278 chip registers
 * Parameter    : fre->Frequency (Range: 137,000,000 ~ 525,000,000 kHz)
**********************************************************************/
void mid_sx1278_SentFreConfig(uint32_t fre)
{
    double fredat;
    static un_sx1278Fre sx1278Fre;
    fredat = fre/0.06103515625L;
    sx1278Fre.fredat = (uint32_t)fredat;  
    SX1278WriteReg(LR_RegFrMsb,sx1278Fre.Fre[1]); 
    SX1278WriteReg(LR_RegFrMid,sx1278Fre.Fre[2]); 
    SX1278WriteReg(LR_RegFrLsb,sx1278Fre.Fre[3]);  
}


/*********************************************************************
 * Name         : mid_SX1278_init
 * Function     : SX1278 Initialize
 * Parameter    : None
**********************************************************************/
void mid_SX1278_init(void)
{
  loraRegPara.CommodeVar.loraPower = LORAPOWER_16DBM;  
  loraRegPara.CommodeVar.loraFre.fredat = 470000;
  loraRegPara.CommodeVar.loraSf = LORASF_7; 
  loraRegPara.CommodeVar.loraRate = LORA_RATE_250KHZ; 
  loraRegPara.CommodeVar.loraCr = LORA_CR_4_5; 
  lorarxDatLen = 0;

  mid_SX1278_LoRaRxConfig(loraRegPara);
}


/*********************************************************************
 * Name         : mid_sx1278_Config
 * Function     : Configure SX1278 registers
 * Parameter    : loraPara->Configuration parameters for the registers
**********************************************************************/
static void mid_sx1278_Config(un_LoraPara loraPara)
{
    uint8_t temp;
    mid_sx1278_Rest();                 

    SX1278WriteReg(LR_RegOpMode,0x00); 
    mid_1278_delay(100);
    SX1278WriteReg(REG_LR_TCXO,0x09); 
   
    SX1278WriteReg(LR_RegOpMode,0x88); 
    
    mid_sx1278_SentFreConfig(loraPara.CommodeVar.loraFre.fredat);  
    
    mid_sx1278_SentPowerConfig(loraPara.CommodeVar.loraPower);
    
    
    SX1278WriteReg(LR_RegOcp,0x0B);    
    SX1278WriteReg(LR_RegLna,0x23);    

    
    Sx1278_SpreadFat = loraPara.CommodeVar.loraSf;
    if(loraPara.CommodeVar.loraSf==6)  //SFactor=6
    {
      SX1278WriteReg(LR_RegModemConfig1,((loraPara.CommodeVar.loraRate<<4)+(loraPara.CommodeVar.loraCr<<1)+0x01));//Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
      SX1278WriteReg(LR_RegModemConfig2,((loraPara.CommodeVar.loraSf<<4)+(Sx1278_CRC<<2)+0x03));
      //  0110 0000
      temp = SX1278ReadReg(LR_RegPacketConfig2);
      temp &= 0xF8;
      temp |= 0x05;  //b101
      SX1278WriteReg(LR_RegPacketConfig2,temp); 
      
      
      SX1278WriteReg(LR_RegSeqConfig2,0x0C); 
    } 
    else
    {
      SX1278WriteReg(LR_RegModemConfig1,((loraPara.CommodeVar.loraRate<<4)+(loraPara.CommodeVar.loraCr<<1)+0x00));//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
      SX1278WriteReg(LR_RegModemConfig2,((loraPara.CommodeVar.loraSf<<4)+(Sx1278_CRC<<2)+0x03));  //SFactor &  LNA gain set by the internal AGC loop 
      SX1278WriteReg(LR_RegModemConfig3,0x08);//LowDataRateOptimize en
      
      temp = SX1278ReadReg(LR_RegPacketConfig2);
      temp &= 0xF8;   //  0b 0xfa   1111 1010 
                                //& 1111 1000    
      temp |= 0x03;  ///b101   
      SX1278WriteReg(LR_RegPacketConfig2,temp); 
      SX1278WriteReg(LR_RegSeqConfig2,0x0A);     
      
    }
    
    SX1278WriteReg(LR_RegSymbTimeoutLsb,0xFF);  
    SX1278WriteReg(LR_RegPreambleMsb, 0);  
    SX1278WriteReg(LR_RegPreambleLsb,16);  
    
    //11    00 01 10 11
    SX1278WriteReg(REG_LR_DIOMAPPING2,0x00); 
    SX1278WriteReg(LR_RegOpMode,0x89);       //Entry standby mode                                     
}


/**********************************************************
 * Name         : mid_Sx1278_LoRaEntryTxConfig
 * Function     : Configure registers for SX1278 wireless transmission
 * Parameter    : loraPara->parameters
**********************************************************/
static void mid_Sx1278_LoRaEntryTxConfig(un_LoraPara loraPara)
{
    uint8_t addr;
    
    mid_sx1278_Config(loraPara); 
    
    SX1278WriteReg(LR_RegHopPeriod,0x00);          
    SX1278WriteReg(REG_LR_DIOMAPPING1,0x40);     
    
    SX1278WriteReg(LR_RegIrqFlags,0xFF);        
    SX1278WriteReg(LR_RegIrqFlagsMask,0xF7);      //Open TxDone interrupt
    
    addr = SX1278ReadReg(LR_RegFifoTxBaseAddr);   //RegFiFoTxBaseAddr   
    SX1278WriteReg(LR_RegFifoAddrPtr,addr); 
}

/**********************************************************
 * Name         : mid_Sx1278_LoRaSentBuffer
 * Function     : send specified length of data to SX1278 module
 * Parameter    : dat-> point to the data to send 
 *                len-> data length
**********************************************************/
void mid_Sx1278_LoRaSentBuffer(unsigned char *dat, unsigned char len)
{ 
    mid_Sx1278TxGpioConfig();
    
    mid_Sx1278_LoRaEntryTxConfig(loraRegPara); 
    mid_1278_delay(100);
    
    SX1278WriteReg(LR_RegPayloadLength,len);   
    
    sx1278WriteBuffer(0,dat,len); 
    
    SX1278WriteReg(LR_RegOpMode,0x8B);
    
    while(!(RF_IRQ_DIO0_DATABIT)); 
    
    
    SX1278WriteReg(LR_RegOpMode,0x8D); 
    mid_SX1278_LoRaRxConfig(loraRegPara);
}


/**********************************************************
 * Name         : mid_Sx1278_RxGpioGonfig
 * Function     : config Lora receiving mode
 * Parameter    : None
**********************************************************/
void mid_Sx1278_RxGpioGonfig(void)
{
    GPIO_Init(RF_IRQ_DIO0_PORT,RF_IRQ_DIO0_PIN, GPIO_Mode_In_FL_IT); 
}

/**********************************************************
 * Name         : mid_Sx1278TxGpioConfig
 * Function     : config Lora to transmit mode(DIO input, disable interrupt)
 * Parameter    : None
**********************************************************/
void mid_Sx1278TxGpioConfig(void)
{
    GPIO_Init(RF_IRQ_DIO0_PORT, RF_IRQ_DIO0_PIN, GPIO_Mode_In_PU_No_IT);  
}

/**********************************************************
 * Name         : SX1278_LoRaRxConfig
 * Function     : config Lora receive mode
 * Parameter    : loraPara->parameters
**********************************************************/
void mid_SX1278_LoRaRxConfig(un_LoraPara loraPara)
{
  uint8_t addr; 
 
  mid_sx1278_Config(loraPara);  

  if(loraPara.CommodeVar.loraSf == LORASF_6)
  {
    SX1278WriteReg(REG_LR_PADAC,0x87); 
  }
  else
    SX1278WriteReg(REG_LR_PADAC,0x84); 
  
  SX1278WriteReg(LR_RegHopPeriod,0);
  SX1278WriteReg(REG_LR_DIOMAPPING1,0x00);
  SX1278WriteReg(LR_RegIrqFlagsMask,0xBF); 
  SX1278WriteReg(LR_RegIrqFlags,0xFF);  
  
  addr = SX1278ReadReg(LR_RegFifoRxBaseAddr); 
  SX1278WriteReg(LR_RegFifoAddrPtr,addr);                        
  SX1278WriteReg(LR_RegOpMode,0x8D);         
  
  mid_Sx1278_RxGpioGonfig();   
}


/**********************************************************
 * Name         : mid_Sx1278RxDateGete
 * Function     : receive Lora wireless data 
 * Parameter    : None
**********************************************************/
void mid_Sx1278RxDateGet(void)
{
    uint8_t addr; 
    
    addr = SX1278ReadReg(LR_RegFifoRxCurrentaddr);  
    SX1278WriteReg(LR_RegFifoAddrPtr, addr);       
    
    if(Sx1278_SpreadFat == 6)
    {
        lorarxDatLen = 21; 
    }
    else
    {
        lorarxDatLen = SX1278ReadReg(LR_RegRxNbBytes);      // SPIRead((u8)(LR_RegRxNbBytes>>8));//Number for received bytes  
    } 
    sx1278ReadBuffer(0x00, &lorarxbuf[0], lorarxDatLen); 
    
    QueueDataIn(LoraRxMsg, &lorarxbuf[0], lorarxDatLen);
    
    SX1278WriteReg(LR_RegIrqFlags,0xFF);  
}

void mid_Sx1278_Sleep(void)
{
    SX1278WriteReg(LR_RegOpMode, 0x08);             // set to Lora mode
}
