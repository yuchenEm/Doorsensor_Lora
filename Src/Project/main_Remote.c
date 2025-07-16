/********************************************************
  * @Name   main.c(Remote)
  * @Brief  
  *******************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm8l10x.h"
#include "hal_key.h"
#include "hal_spi.h"
#include "hal_flash.h"
#include "mid_sx1278.h"
#include "mid_protocal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define LED_GPIO_PORT  GPIOA
#define LED_GPIO_PINS  GPIO_Pin_3

__IO unsigned char Systick;

unsigned char BatteryState;		// Battery status，0->normal, 1->BatteryLow
unsigned char bBatteryState;	// Battery status backup
unsigned char BatteryDtcTimer;	
unsigned short SleepTimer;      // enter sleep timer

unsigned short wCRCTalbeAbs[] =
{
	0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,
};

unsigned char UniqueID[12];     // device MAC address
//unsigned char NoteID[2];      // CRC16 NodeNo.

unsigned char systemMode;	    //0->Offline，1->Online
unsigned char netKeyPressFlag;

unsigned short OffLineModeTimer;

typedef struct DtcPara
{
    unsigned char DtcPairFlag;      //sensor/remote pair state flag, 0->not paired yet, 1->paired alread
    unsigned char NodeID[2];        
    unsigned char Reserve;          
}stu_DtcPara_typedef;

stu_DtcPara_typedef stu_DtcPara;

static void TIM_Config(void);
static void app_BatteryDtcTask(void);
static void Delay(unsigned short nCount);
unsigned short api_crc16(unsigned char *ptr, unsigned int wDataLen);

void Key_EventHandler(KEY_VALUE_TYPEDEF KeyValue);
void ProtocalEvent_Handler(unsigned char *data);

static void Online_Mode(void);
static void Offline_Mode(void);

void main(void)
{
    unsigned char i;
    unsigned short uidCrc16;
    
    // initial GPIOs as Low-Consumption mode:
    GPIO_Init(GPIOA, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOB, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOD, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    
    GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_No_IT);
    
    // variables initial:
    BatteryState = 0;
    bBatteryState = 0;
    BatteryDtcTimer = 0;
    
    netKeyPressFlag = 0;
    systemMode = 0;
    OffLineModeTimer = 0;
    stu_DtcPara.DtcPairFlag = 0;
    
    Systick = 0;
    
    SleepTimer = 0;
    
    // prepare CRC16 code from MAC address
    for(i=0; i<12; i++)
    {
        UniqueID[i] = *((__IO uint8_t *)(0x4925 + i));  //get MAC address from RAM address 0x4925
    }
    uidCrc16 = api_crc16(UniqueID, 12);
    stu_DtcPara.NodeID[0] = uidCrc16 & 0xFF;        //CRC low byte
    stu_DtcPara.NodeID[1] = (uidCrc16>>8) & 0xFF;   //CRC high byte
 
    // module initial:
    Hal_Key_Init();
    TIM_Config(); 
    hal_spi_Config();
    mid_SX1278_init(); 
    mid_Protocal_Init();
    
    // call-back function register:
    Hal_Key_KeyScanCBSRegister(Key_EventHandler);
    mid_Protocal_CBSRegister(ProtocalEvent_Handler);
    
    // clock and GPIO initial:
    CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv1);   /* select Clock = 16 MHz */ 
    
    GPIO_Init(LED_GPIO_PORT, LED_GPIO_PINS, GPIO_Mode_Out_PP_Low_Fast);
    GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT);
         
    enableInterrupts(); 
    GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PINS);    
    

    while (1)
    {
        if(Systick)
        {
            Systick = 0;
            
            Hal_Key_Pro();                     
            mid_Protocal_Pro();
            
            if(systemMode == 1)
            {
                Online_Mode();
            }
            else
            {
                Offline_Mode();
            }
            
            if(!Hal_Key_GetAllKeyState())
            {
                SleepTimer++;
            }
            else
            {
                SleepTimer = 0;
            }
            
            if(SleepTimer > 500)
            {
                SleepTimer = 0;
                
                disableInterrupts();
                
                Hal_Key_Init();                  
                mid_Sx1278_Sleep();	
                                         
                GPIO_ResetBits(GPIOA,GPIO_Pin_3);

                BatteryState = 0;
                bBatteryState = 0;

                enableInterrupts(); 
                
                halt();                                     
            }
        }
    }
}

/****************************************************
Function: Key event handler(for call-back register)
Parameters: KeyValue
Return: Null
*****************************************************/
void Key_EventHandler(KEY_VALUE_TYPEDEF KeyValue)
{
    switch(KeyValue)
    {
        case KEY1_CLICK:
            {
                mid_lora_TxDataPackage(LORA_COM_DISARM, stu_DtcPara.NodeID);
            }
            break;
            
        case KEY2_CLICK:
            {
                mid_lora_TxDataPackage(LORA_COM_HOMEARM, stu_DtcPara.NodeID);
            }
            break;
    
        case KEY3_CLICK:
            {
                mid_lora_TxDataPackage(LORA_COM_ALARM, stu_DtcPara.NodeID);
            }
            break;
        
        case KEY4_CLICK:
            {
                mid_lora_TxDataPackage(LORA_COM_AWAYARM, stu_DtcPara.NodeID);
            }
            break;
            
        case KEYNET_LONG_PRESS:
            {
                if(stu_DtcPara.DtcPairFlag == 0x01)
                {
                    stu_DtcPara.DtcPairFlag = 0;
                    
                    hal_flash_WriteDat((unsigned char *)(&stu_DtcPara));
                    hal_flash_ReadDat((unsigned char *)(&stu_DtcPara), sizeof(stu_DtcPara));
                    
                    GPIO_SetBits(GPIOA, GPIO_Pin_3);
                    Delay(2000);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
                }
                
                netKeyPressFlag = 1;
            }
            break;
    }
}

/*********************************************
Function: Lora dataframe protocal handler(for call-back register)
Parameters: *data
Return: Null
**********************************************/
void ProtocalEvent_Handler(unsigned char *data)
{
    en_lora_eventTypedef cmd;
    unsigned char i, error;
    
    error = 0;
    
    cmd = (en_lora_eventTypedef)data[0];
    
    if(cmd == LORA_COM_RESPOSE_APPLY_NET)
    {
        for(i=0; i<12; i++)
        {
            if(data[3+i] != UniqueID[i])
            {
                error = 1;
                break;
            }
        }
        if(!error)
        {
            stu_DtcPara.DtcPairFlag = 1;
            stu_DtcPara.NodeID[0] = data[1];
            stu_DtcPara.NodeID[1] = data[2];
            
            hal_flash_WriteDat((unsigned char *)(&stu_DtcPara));
            hal_flash_ReadDat((unsigned char *)(&stu_DtcPara), sizeof(stu_DtcPara));
        }
    }
}

static void Online_Mode(void)     
{
    app_BatteryDtcTask();
}

static void Offline_Mode(void)    
{
    SleepTimer = 0;
    
    if(netKeyPressFlag == 1)
    { 
        OffLineModeTimer++;
        if((OffLineModeTimer % 1000) == 0)  //send every 1s
        {
            mid_lora_TxDataPackage(LORA_COM_APPLY_NET, stu_DtcPara.NodeID);
        }
        if(OffLineModeTimer > 5000)         //repeat 5 times
        {
            systemMode = 0;
            OffLineModeTimer = 0;
            netKeyPressFlag = 0;
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
        }
    }

    if(stu_DtcPara.DtcPairFlag == 0x01)
    {
        systemMode = 1;
        OffLineModeTimer = 0;
        netKeyPressFlag = 0;
         
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
        Delay(200);	
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);
        Delay(200);
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
        Delay(200);	
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);            
    }
}

static void TIM_Config(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);   
    TIM2_DeInit();                                          
    TIM2_TimeBaseInit(TIM2_Prescaler_16, TIM2_CounterMode_Up, 1000); // Timebase: 1ms
    TIM2_ITConfig(TIM2_IT_Update, ENABLE);   
    TIM2_ARRPreloadConfig(ENABLE);  
    TIM2_Cmd(ENABLE); 
}

static void Delay(unsigned short nCount)
{
    while (nCount != 0)
    {
        nCount--;
    }
}

static void app_BatteryDtcTask(void)
{
	if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0))
	{
		BatteryDtcTimer++;
		if(BatteryDtcTimer > 50)	//50ms
		{
			BatteryDtcTimer = 0;
			if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0))
			{
				BatteryState = 1;
			}
            else
			{
				BatteryState = 0; 
			}
		}
	}
    else
	{
		BatteryState = 0;
		BatteryDtcTimer = 0;	
	}
    
	if(bBatteryState != BatteryState)
	{
		bBatteryState = BatteryState;
		if(BatteryState == 1)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_3);
            mid_lora_TxDataPackage(LORA_COM_BAT_LOW, stu_DtcPara.NodeID);
		}
        else 
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);   
        }
	}

}

/************************************************
Function: CRC16(Modbus) calculation algorithm
Parameters: 
    *ptr:       pointer to the data starting address
    wDataLen:   data length
Return: 
    wCRC:       CRC16 value 0x0000-0xFFFF
*************************************************/
unsigned short api_crc16(unsigned char *ptr, unsigned int wDataLen) 
{
	unsigned short wCRC = 0xFFFF;
	unsigned short i;
	unsigned char chChar;
	unsigned char temp[2];
    
	for (i = 0; i < wDataLen; i++)
	{
		chChar = *ptr++;
		wCRC = wCRCTalbeAbs[(chChar ^ wCRC) & 15] ^ (wCRC >> 4);        // 15 = 0x0F
		wCRC = wCRCTalbeAbs[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
	}
	temp[0] = wCRC&0xFF; 
	temp[1] = (wCRC>>8)&0xFF;
	wCRC = (temp[0]<<8)|temp[1];
	return wCRC;
}


INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_IRQHandler, 19)
{
    Systick = 1;
    TIM2_ClearITPendingBit(TIM2_IT_Update);
}

INTERRUPT_HANDLER(EXTI0_IRQHandler, 8)
{
    EXTI_ClearITPendingBit (EXTI_IT_Pin0);
    mid_Sx1278RxDateGet();
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/