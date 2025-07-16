#include "stm8l10x.h"
#include "hal_usart.h"
#include "mid_usart.h"
#include "queue.h"
#include "hal_flash.h"

stru_SystemVari SystemVari;

static unsigned char app_usart_ComModeChange(unsigned char *buf,unsigned char len);
static void mid_usart_ComAtPro(unsigned char *buf,unsigned char len);

void mid_usart_SystemVariToFac(void)
{
    SystemVari.com_Mode = COM_MODE_GFPT;
    SystemVari.workmode = WORKMODE_NOMAL;
    SystemVari.usart_ipr = USART_IPR_115200;  
    SystemVari.loraPara.loraCr = LORA_CR_4_5;
    SystemVari.loraPara.loraBw = LORA_RATE_250KHZ;
    SystemVari.loraPara.loraFre.fredat = 470000;
    SystemVari.loraPara.loraPower = LORAPOWER_13DBM;
    SystemVari.loraPara.loraSf = LORASF_7;
    
    mid_sx1278Vart(SystemVari.loraPara); // reset parameter
  
    hal_flash_WriteDat((unsigned char *)(&SystemVari));
}

void mid_usart_DecSystemVari(void) // detect systemvariables
{
    unsigned char error = 0; // =1 data in flsh is incorrect
    hal_flash_ReadDat((unsigned char *)(&SystemVari),sizeof(SystemVari));
  
    if((SystemVari.com_Mode != COM_MODE_GFPT) && (SystemVari.com_Mode != COM_MODE_AT))
    {
        SystemVari.com_Mode = COM_MODE_GFPT;
        error = 1;
    }
    if((SystemVari.workmode != WORKMODE_NOMAL)&&(SystemVari.workmode != WORKMODE_LOW))
    {
        SystemVari.workmode = WORKMODE_NOMAL;
        error = 1;
    }
    if((SystemVari.usart_ipr > USART_IPR_115200) || (SystemVari.usart_ipr < USART_IPR_4800))
    {
        SystemVari.usart_ipr = USART_IPR_115200;
        error = 1;
    }  
    if((SystemVari.loraPara.loraCr > LORA_CR_4_8) || (SystemVari.loraPara.loraCr < LORA_CR_4_5))
    {
        SystemVari.loraPara.loraCr = LORA_CR_4_5;
        error = 1;
    }   
    if(SystemVari.loraPara.loraBw > LORA_RATE_500KHZ) 
    {
        SystemVari.loraPara.loraBw= LORA_RATE_250KHZ;
        error = 1;
    }    
    if((SystemVari.loraPara.loraFre.fredat  >  525000) || (SystemVari.loraPara.loraFre.fredat  < 137000))
    {
        SystemVari.loraPara.loraFre.fredat= 470000;
        error = 1;
    }  
    if((SystemVari.loraPara.loraPower < LORAPOWER_2DBM) || ((SystemVari.loraPara.loraPower > LORAPOWER_17DBM) && (SystemVari.loraPara.loraPower != LORAPOWER_20DBM)))
    {
        SystemVari.loraPara.loraPower = LORAPOWER_13DBM;
        error = 1;
    }    
    if((SystemVari.loraPara.loraSf > LORASF_12) || (SystemVari.loraPara.loraSf < LORASF_6))
    {
        SystemVari.loraPara.loraSf= LORASF_7;
        error = 1;
    }
    
    if(error == 1)
    {
        hal_flash_WriteDat((unsigned char *)(&SystemVari));
    }     
}


void mid_usart_init(void)
{
    mid_usart_DecSystemVari();
    
/*    SystemVari.com_Mode = COM_MODE_GFPT; 
    SystemVari.usart_ipr = USART_IPR_115200; // baud rate
    SystemVari.loraPara.loraCr = LORA_CR_4_5; //CRC: Cyclic Redundancy Checksum
    SystemVari.loraPara.loraBw = LORA_RATE_250KHZ; // bandwidth
    SystemVari.loraPara.loraFre.fredat = 470000; // Freq
    SystemVari.loraPara.loraPower = LORAPOWER_13DBM; //power
    SystemVari.loraPara.loraSf = LORASF_7; // Spreading Factor 
*/
    
    mid_sx1278Vart(SystemVari.loraPara);
    HAL_USART_Config(SystemVari.usart_ipr); 
}

void mid_usart_Pro(void)
{
    uint8_t len, i;
    uint8_t USART_RxBuffer[256];
    len = QueueDataLen(USART_RxQueue);
    if(len)
    {
        TIM_USART_GetNewDataDelay++;
        if(TIM_USART_GetNewDataDelay > 40)
        {
            TIM_USART_GetNewDataDelay = 0;
            for(i=0;i<len;i++)
            {
                QueueDataOut(USART_RxQueue, &USART_RxBuffer[i]);
            }
            if((len > 6)&&(app_usart_ComModeChange(&USART_RxBuffer[0],len)) == 1)
            {
                //hal_flash_SaveDat(&falshDat.CommodeVar.flag);
                HAL_USART_PutInTxQueue("SetOk\n\r",7);  
                return;
            }  
        
        
            switch(SystemVari.com_Mode)
            {
                case COM_MODE_GFPT:
                {
                    mid_Sx1278_LoRaSentBuffer(USART_RxBuffer, len);
                    break;
                }
                case COM_MODE_AT:
                {
                    mid_usart_ComAtPro(USART_RxBuffer,len);
                    break;
                }
            }
            
            //mid_Sx1278_LoRaSentBuffer(USART_RxBuffer, len);
        }
    }
}

static uint32_t fuc_SetFredat(unsigned char *buf,unsigned char len)
{
    uint32_t datFre;
    uint8_t i;
    datFre = 0;
    for(i=0;i<len;i++)
    {
        if((*buf >='0') && (*buf<='9')) 
        {       
            datFre *= 10; 
            datFre += (*buf - '0'); 
            buf ++;
        }
        else if(*buf ==0x0D)
        {
            break;
        }
        else
        {
            datFre = 0;
        }
    }
    return datFre;
}



static void mid_usart_ComAtPro(unsigned char *buf, unsigned char len)
{
    uint32_t datFre;
    unsigned char dat;
    unsigned char error; // 0: ok, 1: error
    error = 1;
    
    // AT_mode set baud rate:
    //    USART_IPR_4800 = 1,
    //    USART_IPR_9600,    
    //    USART_IPR_14400,    
    //    USART_IPR_19200,
    //    USART_IPR_28800,    
    //    USART_IPR_38400,     
    //    USART_IPR_57600,
    //    USART_IPR_115200, 
    //    USART_IPR_MAX = 8,    
    if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'I') && (buf[4] == 'P') && (buf[5] == 'R') && (buf[6] == '='))
    { // set baud rate
        if((buf[7]>'0') && (buf[7]<'9') && (buf[8] == 0x0D)&& (buf[9] == 0x0A)) 
        {
            SystemVari.usart_ipr = (en_usart_ipr)(buf[7] - '0');
            //HAL_USART_PutInTxQueue("OK\n\r",4); 
            HAL_USART_SendStrLen("OK\n\r", 4);
            HAL_USART_Config(SystemVari.usart_ipr);
            error = 0;
        }
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7); 
        }
    }

    // AT_mode set LoraPower:
    //    LORAPOWER_2DBM = 2,
    //    LORAPOWER_3DBM,
    //    LORAPOWER_4DBM,    
    //    LORAPOWER_5DBM,
    //    LORAPOWER_6DBM,
    //    LORAPOWER_7DBM,
    //    LORAPOWER_8DBM,    
    //    LORAPOWER_9DBM,
    //    LORAPOWER_10DBM,
    //    LORAPOWER_11DBM,
    //    LORAPOWER_12DBM,    
    //    LORAPOWER_13DBM,
    //    LORAPOWER_14DBM,    
    //    LORAPOWER_15DBM,
    //    LORAPOWER_16DBM,
    //    LORAPOWER_17DBM,
    //    LORAPOWER_20DBM = 20,     
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'P') && (buf[4] == 'O') && (buf[5] == 'W') && (buf[6] == 'E')&& (buf[7] == 'R')&& (buf[8] == '='))
    {///发送功率配置 LORAPOWER = 2 ~ 17, 20 dBm
        if((buf[9] > '1') && (buf[9] <= '9')) 
        {
            dat = buf[9] - '0';
            if(dat == 2)
            {
                if((buf[10] == '0')&&(buf[11] == 0x0D))
                {      
                    SystemVari.loraPara.loraPower = LORAPOWER_20DBM;
                    error = 0;
                }
                else if(buf[10] == 0x0D)
                {
                    SystemVari.loraPara.loraPower = LORAPOWER_2DBM;    
                    error = 0;
                }
            }
            else if(buf[10] == 0x0D)
            {
                SystemVari.loraPara.loraPower = (en_LoraPower)dat;
                error = 0;
            }
        }
        else if(buf[9] == '1')
        {
            //dat = 1;
            if(((buf[10] >= '0') && (buf[10] < '8'))&&(buf[11]==0x0D)) 
            {
                dat = 10;
                dat += (buf[10] - '0');
                SystemVari.loraPara.loraPower =  (en_LoraPower)dat;
                //HAL_USART_PutInTxQueue("OK\n\r", 4);
                error = 0;
            }
        }
        
        if(error == 0)
        {
            HAL_USART_PutInTxQueue("OK\n\r",4);
            mid_sx1278Vart(SystemVari.loraPara);
            mid_SX1278_LoRaRxConfig();
            error = 0;
        }
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7); 
        }
    }
    
    // AT_mode set Freq:   
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'F') && (buf[4] == 'R') && (buf[5] == 'E') && (buf[6] == '='))  
    {
        datFre = fuc_SetFredat(&buf[7],6);
        if((datFre> 137000)&&(datFre< 525000))
        {
            SystemVari.loraPara.loraFre.fredat = datFre;
            error = 0;
        }     
        if(error == 0)
        {
            HAL_USART_PutInTxQueue("OK\n\r",4);
            mid_sx1278Vart(SystemVari.loraPara);
            mid_SX1278_LoRaRxConfig();  
            error = 0;
        }
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7); 
        }
    }
    
    // AT_mode set SF:
    //    LORASF_6 = 6,
    //    LORASF_7,    
    //    LORASF_8,    
    //    LORASF_9,    
    //    LORASF_10,
    //    LORASF_11,    
    //    LORASF_12, =12     
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'S') && (buf[4] == 'F')&& (buf[5] == '='))  
    {//扩频因子
        dat = 0;
    
        if((buf[6]>='0') && (buf[6]<='9')) 
        {  
            dat = buf[6] - '0';
        }
    
        if(((buf[7]>='0') && (buf[7]<='2')) && (buf[8] == 0x0D)) 
        {  
            dat *= 10;
            dat += buf[7] - '0';
        } 
        else if(buf[7] != 0x0D) 
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7); 
            return;
        }
 
        if((dat > 5) && (dat < 13)) 
        {
            SystemVari.loraPara.loraSf = (en_LoraSf)dat; 
            mid_sx1278Vart(SystemVari.loraPara);
            mid_SX1278_LoRaRxConfig(); 
            HAL_USART_PutInTxQueue("OK\n\r",4);
            error = 0;
        }  
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7); 
        }    
    }


    // AT_mode set BW:
    //    LORA_RATE_7_8KHZ, =0
    //    LORA_RATE_10_4KHZ,
    //    LORA_RATE_15_6KHZ,
    //    LORA_RATE_20_8KHZ,
    //    LORA_RATE_31_25KHZ,
    //    LORA_RATE_41_7KHZ,
    //    LORA_RATE_62_5KHZ,
    //    LORA_RATE_125KHZ,
    //    LORA_RATE_250KHZ,
    //    LORA_RATE_500KHZ, =9    
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'R') && (buf[4] == 'A') && (buf[5] == 'T') && (buf[6] == 'E') && (buf[7] == '='))  
    {
        if(((buf[8]>='0') && (buf[8]<='9'))  && (buf[9] == 0x0D)) 
        {  
            SystemVari.loraPara.loraBw = (en_LoraBw)(buf[8] - '0');
            mid_sx1278Vart(SystemVari.loraPara);
            mid_SX1278_LoRaRxConfig(); 
      
            HAL_USART_PutInTxQueue("OK\n\r",4);
            error = 0;
        } 
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7); 
        }
    } 

    // AT_mdoe set CR: 
    //    LORA_CR_4_5 = 1,
    //    LORA_CR_4_6,
    //    LORA_CR_4_7,
    //    LORA_CR_4_8, =4
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'C') && (buf[4] == 'R')&& (buf[5] == '='))    
    {
        if(((buf[6]>='1') && (buf[6]<='4')) && (buf[7] == 0x0D))  
        {  
            SystemVari.loraPara.loraCr= (en_LoraCr)(buf[6] - '0'); 
            mid_sx1278Vart(SystemVari.loraPara);
            mid_SX1278_LoRaRxConfig(); 
            HAL_USART_PutInTxQueue("OK\n\r",4);
            error = 0;
        }
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7);
        }
    }

    // AT_mode reset default setting:    
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'R') && (buf[4] == 'E') && (buf[5] == 'S') && (buf[6] == 'T') && (buf[7] == 0x0D))  
    {
        mid_usart_SystemVariToFac();
        HAL_USART_SendStrLen("OK\n\r",4);
        HAL_USART_Config(SystemVari.usart_ipr);
        mid_SX1278_LoRaRxConfig();
    }
    
    // AT_mode send current parameters:
    //SystemVari.com_Mode = COM_MODE_GFPT;
    //SystemVari.workmode = WORKMODE_NOMAL;
    //SystemVari.usart_ipr = USART_IPR_115200;  
    //SystemVari.loraPara.loraCr = LORA_CR_4_5;
    //SystemVari.loraPara.loraBw = LORA_RATE_250KHZ;
    //SystemVari.loraPara.loraFre.fredat = 470000;
    //SystemVari.loraPara.loraPower = LORAPOWER_13DBM;
    //SystemVari.loraPara.loraSf = LORASF_7;
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'P') && (buf[4] == 'A') && (buf[5] == 'R') && (buf[6] == 'A') && (buf[7] == 0x0D))
    {
        hal_flash_ReadDat((unsigned char *)(&SystemVari),sizeof(SystemVari));
        if(SystemVari.com_Mode == COM_MODE_GFPT)
        {
            HAL_USART_SendStrLen("Mode:GFPT\n\r",11);
        }
        if(SystemVari.com_Mode == COM_MODE_AT)
        {
            HAL_USART_SendStrLen("Mode:AT\n\r",9);
        }
        if(SystemVari.workmode == WORKMODE_NOMAL)
        {
            HAL_USART_SendStrLen("WorkMode:NORMAL\n\r",17);
        }
        if(SystemVari.workmode == WORKMODE_LOW)
        {
            HAL_USART_SendStrLen("WorkMode:LOW\n\r",14);
        }
        if(SystemVari.usart_ipr == USART_IPR_4800)
        {
            HAL_USART_SendStrLen("BaudRate:4800\n\r",15);
        }
        if(SystemVari.usart_ipr == USART_IPR_9600)
        {
            HAL_USART_SendStrLen("BaudRate:9600\n\r",15);
        }
        if(SystemVari.usart_ipr == USART_IPR_14400)
        {
            HAL_USART_SendStrLen("BaudRate:14400\n\r",16);
        }
        if(SystemVari.usart_ipr == USART_IPR_19200)
        {
            HAL_USART_SendStrLen("BaudRate:19200\n\r",16);
        }
        if(SystemVari.usart_ipr == USART_IPR_28800)
        {
            HAL_USART_SendStrLen("BaudRate:28800\n\r",16);
        }
        if(SystemVari.usart_ipr == USART_IPR_38400)
        {
            HAL_USART_SendStrLen("BaudRate:38400\n\r",16);
        }
        if(SystemVari.usart_ipr == USART_IPR_57600)
        {
            HAL_USART_SendStrLen("BaudRate:57600\n\r",16);
        }
        if(SystemVari.usart_ipr == USART_IPR_115200)
        {
            HAL_USART_SendStrLen("BaudRate:115200\n\r",17);
        }
        if(SystemVari.loraPara.loraCr == LORA_CR_4_5)
        {
            HAL_USART_SendStrLen("CodingRate:4/5\n\r",16);
        }
        if(SystemVari.loraPara.loraCr == LORA_CR_4_6)
        {
            HAL_USART_SendStrLen("CodingRate:4/6\n\r",16);
        }
        if(SystemVari.loraPara.loraCr == LORA_CR_4_7)
        {
            HAL_USART_SendStrLen("CodingRate:4/7\n\r",16);
        }
        if(SystemVari.loraPara.loraCr == LORA_CR_4_8)
        {
            HAL_USART_SendStrLen("CodingRate:4/8\n\r",16);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_7_8KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:7.8KHz\n\r",18);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_10_4KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:10.4KHz\n\r",19);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_15_6KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:15.6KHz\n\r",19);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_20_8KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:20.8KHz\n\r",19);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_31_25KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:31.25KHz\n\r",20);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_41_7KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:41.7KHz\n\r",19);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_62_5KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:62.5KHz\n\r",19);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_125KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:125KHz\n\r",18);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_250KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:250KHz\n\r",18);
        }
        if(SystemVari.loraPara.loraBw == LORA_RATE_500KHZ)
        {
            HAL_USART_SendStrLen("BandWidth:500KHz\n\r",18);
        }
        
        if(SystemVari.loraPara.loraPower == LORAPOWER_2DBM)
        {
            HAL_USART_SendStrLen("Power:2dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_3DBM)
        {
            HAL_USART_SendStrLen("Power:3dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_4DBM)
        {
            HAL_USART_SendStrLen("Power:4dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_5DBM)
        {
            HAL_USART_SendStrLen("Power:5dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_6DBM)
        {
            HAL_USART_SendStrLen("Power:6dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_7DBM)
        {
            HAL_USART_SendStrLen("Power:7dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_8DBM)
        {
            HAL_USART_SendStrLen("Power:8dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_9DBM)
        {
            HAL_USART_SendStrLen("Power:9dBm\n\r",12);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_10DBM)
        {
            HAL_USART_SendStrLen("Power:10dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_11DBM)
        {
            HAL_USART_SendStrLen("Power:11dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_12DBM)
        {
            HAL_USART_SendStrLen("Power:12dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_13DBM)
        {
            HAL_USART_SendStrLen("Power:13dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_14DBM)
        {
            HAL_USART_SendStrLen("Power:14dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_15DBM)
        {
            HAL_USART_SendStrLen("Power:15dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_16DBM)
        {
            HAL_USART_SendStrLen("Power:16dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_17DBM)
        {
            HAL_USART_SendStrLen("Power:17dBm\n\r",13);
        }
        if(SystemVari.loraPara.loraPower == LORAPOWER_20DBM)
        {
            HAL_USART_SendStrLen("Power:20dBm\n\r",13);
        }
    }

    // AT_mode set working mode:
    //WORKMODE_NOMAL = 0;
    //WORKMODE_LOW = 1;
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[02] == '+') && (buf[3] == 'M') && (buf[4] == 'O') && (buf[5] == 'D') && (buf[6] == 'E')&& (buf[7] == '=')&& (buf[9] == 0x0D))
    {
        if((buf[8]>='0') && (buf[8]<'2')) 
        {
            SystemVari.workmode = (en_workmode)(buf[8] - 0X30);
            HAL_USART_SendStrLen("OK\n\r",4);
            error = 0;
        }
        else
        {
            HAL_USART_PutInTxQueue("ERROR\n\r",7);
        }
  }

    // AT_mode send data:    
    else if((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == '+') && (buf[3] == 'D') && (buf[4] == 'A') && (buf[5] == 'T')&& (buf[6] == '='))  
    {
        mid_Sx1278_LoRaSentBuffer(&buf[7],(len-7)); 
        HAL_USART_PutInTxQueue("OK\n\r",4);
    }
    
    if(error == 0)
    {
        hal_flash_WriteDat((unsigned char*)(&SystemVari));
    }
}

static unsigned char app_usart_ComModeChange(unsigned char *buf,unsigned char len)
{
    if((buf[0] == '#') && (buf[1] == 'A') && (buf[2] == 'T') && (buf[3] == 'M') && (buf[5] == 0x0D) && (buf[6] == 0x0A))
    {
        if((buf[4]>='0') && (buf[4]<='1'))   
        {
            SystemVari.com_Mode = (en_commode)(buf[4] - '0');
            hal_flash_WriteDat((unsigned char*)(&SystemVari));
            return 1; // 1: mode change succeed
        } 
    }
    return 0; // 0: mode change fail
}
