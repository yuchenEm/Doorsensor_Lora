/********************************************************
  * @Name   main.c(Doorsensor)
  * @Brief  
  *******************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm8l10x.h"
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
__IO unsigned char DoorInterruptTriggerFlag;    // 0-not trigger, 1-trigger
__IO unsigned char NetKeyInterruptTriggerFlag;
__IO unsigned char AWUInterruptTriggerFlag;

unsigned char BatteryState;		// Battery status，0->normal, 1->BatteryLow
unsigned char bBatteryState;	// Battery status backup
unsigned char BatteryDtcTimer;	
unsigned short SleepTimer;      // enter sleep timer
unsigned short HeartBeatTimer;

unsigned short LoraTxCmdTimer;	  // Tx timer
unsigned char LoraTxCmdCount;     // Tx counter
unsigned char LoraTxCmdBusy;      // TxBusy flag: 0->idle, 1->busy
unsigned char LoraTxCmdResponse;  // returned function code from Host
unsigned char LoraTxCmd;          // sendout function code

unsigned short wCRCTalbeAbs[] =
{
	0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,
};

unsigned char UniqueID[12];       // device MAC address
//unsigned char NoteID[2];        // CRC16 NodeNo.

unsigned char systemMode;	      //0->Offline，1->Online
unsigned char netKeyPressFlag;
unsigned short OffLineModeTimer;

Queue8 LoraTxCmdMsg;              //lora Tx command buffer

typedef struct DtcPara
{
    unsigned char DtcPairFlag;      //sensor/remote pair state flag, 0->not paired yet, 1->paired alread
    unsigned char NodeID[2];        
    unsigned char Reserve;          
}stu_DtcPara_typedef;

stu_DtcPara_typedef stu_DtcPara;

static void TIM_Config(void);
static void AWUInit(void);
static void app_BatteryDtcTask(void);
static void app_DoorDtcTask(void);
static void app_NetKeyScanTask(void);
static void app_LoraTxCmdTask(void);
static void Delay(unsigned short nCount);
unsigned short api_crc16(unsigned char *ptr, unsigned int wDataLen);
static void LoraTxMsgInput(en_lora_eventTypedef cmd, unsigned char emptyFlag);

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
    
    GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_IT);
    EXTI_SetPinSensitivity(EXTI_Pin_2,EXTI_Trigger_Rising_Falling);
    
    // variables initial:
    BatteryState = 0;
    bBatteryState = 0;
    BatteryDtcTimer = 0;
    
    netKeyPressFlag = 0;
    systemMode = 0;
    OffLineModeTimer = 0;
    stu_DtcPara.DtcPairFlag = 0;
    
    Systick = 0;
    DoorInterruptTriggerFlag = 0;
    NetKeyInterruptTriggerFlag = 0;
    AWUInterruptTriggerFlag = 0;
    
    SleepTimer = 0;
    HeartBeatTimer = 0;
    
    // prepare CRC16 code from MAC address
    for(i=0; i<12; i++)
    {
        UniqueID[i] = *((__IO uint8_t *)(0x4925 + i));  //get MAC address from RAM address 0x4925
    }
    uidCrc16 = api_crc16(UniqueID, 12);
    stu_DtcPara.NodeID[0] = uidCrc16 & 0xFF;        //CRC low byte
    stu_DtcPara.NodeID[1] = (uidCrc16>>8) & 0xFF;   //CRC high byte
 
    // module initial:
    TIM_Config(); 
    hal_spi_Config();
    mid_SX1278_init(); 
    mid_Protocal_Init();
    AWUInit();
    
    // initial Queue LoraTxCmdMsg
    QueueEmpty(LoraTxCmdMsg);
    
    // call-back function register:
    mid_Protocal_CBSRegister(ProtocalEvent_Handler);
    
    // clock and GPIO initial:
    CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv1);   /* select Clock = 16 MHz */ 
    
    GPIO_Init(LED_GPIO_PORT, LED_GPIO_PINS, GPIO_Mode_Out_PP_Low_Fast);
    GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT);
    
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);       
    EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Falling); 
    GPIO_Init(GPIOC,GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);
         
    enableInterrupts(); 
    GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PINS);    
    
    while (1)
    {
        if(Systick)
        {
            Systick = 0;
                               
            mid_Protocal_Pro();
            app_DoorDtcTask();
            app_NetKeyScanTask();
            app_BatteryDtcTask();
            app_LoraTxCmdTask();
            
            if(systemMode == 1)
            {
                Online_Mode();
            }
            else
            {
                Offline_Mode();
            }
            
            if((!QueueDataLen(LoraTxCmdMsg)) && (!LoraTxCmdBusy))
            {
                SleepTimer++; 
            }
            else
            {
                SleepTimer = 0;
            }
            
            if(SleepTimer > 100)
            {
                disableInterrupts();
                
                SleepTimer = 0;
                
                mid_Sx1278_Sleep();
                
                GPIO_ResetBits(GPIOA, GPIO_Pin_3);
                
                BatteryDtcTimer = 0;	
                DoorInterruptTriggerFlag = 0;
                NetKeyInterruptTriggerFlag = 0;
                
                GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);
                EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Falling);
                
                GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_IT);
                EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Rising_Falling); 
            
                enableInterrupts(); 
                
                halt();
                
                if((stu_DtcPara.DtcPairFlag == 1) && (AWUInterruptTriggerFlag))
                {
                    AWUInterruptTriggerFlag = 0;
                    
                    HeartBeatTimer++;
                    
                    if(HeartBeatTimer > 0)
                    {
                        HeartBeatTimer = 0;
                        
                        if(BatteryState)
                        {
                            LoraTxMsgInput(LORA_COM_BAT_LOW, 0);
                        }
                        else
                        {
                            LoraTxMsgInput(LORA_COM_HEART, 0);
                        }
                    }
                    
                }
                
            }
        }
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
    else if((stu_DtcPara.NodeID[0] == data[1]) 
			&& (stu_DtcPara.NodeID[1] == data[2]))
    {
        //LORA_COM_RESPOSE_HEART     responseCode  0x82
		//LORA_COM_RESPOSE_BAT_LOW   responseCode  0x83
		//LORA_COM_RESPOSE_ALARM     responseCode  0x84
		//LORA_COM_RESPOSE_DOORCLOSE responseCode  0x89
		if((cmd == LORA_COM_RESPOSE_HEART)
			||(cmd == LORA_COM_RESPOSE_BAT_LOW)
			||(cmd == LORA_COM_RESPOSE_ALARM)
			||(cmd == LORA_COM_RESPOSE_DOORCLOSE))
		{
			LoraTxCmdResponse = cmd;
		}
    }
}

/****************************************************************************
Function: detect the door-sensor status, send cooresponding lora message
Parameters: Null
Return: Null
*****************************************************************************/
static void app_DoorDtcTask(void)
{
    if(DoorInterruptTriggerFlag)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))    // high-door open, low-door closed
        {
            Delay(200);
            
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))
            {
                DoorInterruptTriggerFlag = 0;
                
                if(stu_DtcPara.DtcPairFlag == 0)        // offline mode
                {
                    mid_lora_TxDataPackage(LORA_COM_ALARM, stu_DtcPara.NodeID);
                }
                else if(stu_DtcPara.DtcPairFlag == 1)   // online mode
                {
                    LoraTxMsgInput(LORA_COM_ALARM, 0);
                }
            }
            else
            {
                DoorInterruptTriggerFlag = 0;
            }
            
            GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_IT);
        }
        else if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))
        {
            Delay(200);
            
            if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))
            {
                DoorInterruptTriggerFlag = 0;
                
                if(stu_DtcPara.DtcPairFlag == 0) // offline mode
                {
                    mid_lora_TxDataPackage(LORA_COM_DOORCLOSE, stu_DtcPara.NodeID);
                }
                else if(stu_DtcPara.DtcPairFlag == 1) // online mode
                {
                    LoraTxMsgInput(LORA_COM_DOORCLOSE, 0);
                }
            }
            else
            {
                DoorInterruptTriggerFlag = 0;
            }
            
            GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_IT);
        }
        else
		{
		  DoorInterruptTriggerFlag = 0;
		  GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_IT);
		}
    }
}

/****************************************
Function: detect Key1 and Key3 NetPress
Para: Null
Return: Null
*****************************************/
static void app_NetKeyScanTask(void)
{
    if(NetKeyInterruptTriggerFlag)
    {
        if((!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)) && (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)))
        {
            Delay(200);
            SleepTimer = 0;
            
            if((!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)) && (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)))
            {
                NetKeyInterruptTriggerFlag = 0;
                netKeyPressFlag = 1; //Key1 and Key3 pressed
                
                if(stu_DtcPara.DtcPairFlag == 0x01)
				{
					stu_DtcPara.DtcPairFlag = 0;	
                    
					GPIO_SetBits(GPIOA,GPIO_Pin_3); 
                    
					hal_flash_WriteDat((unsigned char *)(&stu_DtcPara));
					hal_flash_ReadDat((unsigned char *)(&stu_DtcPara), sizeof(stu_DtcPara));
					
                    Delay(2000);
                             
					GPIO_ResetBits(GPIOA,GPIO_Pin_3);               
				}
                
                OffLineModeTimer = 0;
            }
            else
            {
                NetKeyInterruptTriggerFlag = 0;
                netKeyPressFlag = 0;
            }
        }

    }
}

/************************************************************
Function: send sensor cmd code to terminal when it's idle
Para: Null
Return: Null
*************************************************************/
static void app_LoraTxCmdTask(void)
{
    if(QueueDataLen(LoraTxCmdMsg) && (!LoraTxCmdBusy))
    {
        LoraTxCmdTimer = 0;	    
		LoraTxCmdCount = 0;     
		LoraTxCmdBusy = 1;      
		LoraTxCmdResponse = 0;   
		
        QueueDataOut(LoraTxCmdMsg, &LoraTxCmd);
        
        mid_lora_TxDataPackage((en_lora_eventTypedef)LoraTxCmd, stu_DtcPara.NodeID);
        
        if(LoraTxCmdBusy)
        {
            if(LoraTxCmdResponse == (LoraTxCmd + 0x80))  //check CmdRespond from terminal "code+ 0x80"
            {                                            //sensor and terminal connection succeed
                LoraTxCmdBusy = 0; 
                HeartBeatTimer = 0;                      //reset heartbeat timer
            }
            else    // connection fail, retry another 4 times
            {
                LoraTxCmdTimer++;
                
                if(LoraTxCmdTimer > 500)
                {
                    LoraTxCmdTimer = 0;
                    
                    mid_lora_TxDataPackage((en_lora_eventTypedef)LoraTxCmd, stu_DtcPara.NodeID);
                    
                    LoraTxCmdCount++;
                    
                    if(LoraTxCmdCount >= 4)
                    {
                        LoraTxCmdCount = 0;
                        LoraTxCmdTimer = 0;
                        LoraTxCmdBusy = 0;
                    }
                }
            }
        }
    }
}

/*********************************************
Function: queue-in sensor lora command byte
Parameters:
    cmd:        command code
    emptyFlag:  1->empty Queue
Return: Null
**********************************************/
static void LoraTxMsgInput(en_lora_eventTypedef cmd, unsigned char emptyFlag)
{
    unsigned char temp;
    
	if(cmd <= LORA_COM_DOORCLOSE)
	{
		temp = cmd;
        
		if(emptyFlag)
		{
			QueueEmpty(LoraTxCmdMsg);
		}
		QueueDataIn(LoraTxCmdMsg, &temp, 1);
	}
}

static void Online_Mode(void)     
{
//    app_BatteryDtcTask();
}

static void Offline_Mode(void)   
{   
    if(netKeyPressFlag == 1)
    { 
        SleepTimer = 0;
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
            GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);
        }
    }

    if(stu_DtcPara.DtcPairFlag == 0x01)
    {
        systemMode = 1;
        OffLineModeTimer = 0;
        netKeyPressFlag = 0;
        
        GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);
         
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
        Delay(200);	
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);
        Delay(200);
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
        Delay(200);	
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);            
    }
}

/******************************************************
Function: Auto wake up configuration
*******************************************************/
static void AWUInit(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_AWU, ENABLE);
    AWU_DeInit();
    AWU_Init(AWU_Timebase_12s);
    AWU_Cmd(ENABLE);
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
            if(stu_DtcPara.DtcPairFlag == 0)        // offline mode
            {
                mid_lora_TxDataPackage(LORA_COM_BAT_LOW, stu_DtcPara.NodeID);
            }
            else if(stu_DtcPara.DtcPairFlag == 1)   // online moode
            {
                LoraTxMsgInput(LORA_COM_BAT_LOW, 0);
            }
			GPIO_SetBits(GPIOA, GPIO_Pin_3);
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
    EXTI_ClearITPendingBit(EXTI_IT_Pin0);
    mid_Sx1278RxDateGet();
}

INTERRUPT_HANDLER(EXTI2_IRQHandler, 10)
{
    DoorInterruptTriggerFlag = 1;
    GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_FL_No_IT);
    EXTI_ClearITPendingBit(EXTI_IT_Pin2);
}

INTERRUPT_HANDLER(EXTI4_IRQHandler, 12)
{
    NetKeyInterruptTriggerFlag = 1;
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_No_IT);
    EXTI_ClearITPendingBit (EXTI_IT_Pin4);
}

INTERRUPT_HANDLER(AWU_IRQHandler, 4)
{
    AWU_GetFlagStatus();
    AWUInterruptTriggerFlag = 1;
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