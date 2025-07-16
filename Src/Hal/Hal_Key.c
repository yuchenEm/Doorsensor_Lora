/****************************************************
  * @Name	Hal_Key.c
  * @Brief	Key scan module.
*****************************************************/

#include "stm8l10x.h"
#include "hal_key.h"

static unsigned char Hal_Key_GetKey1Sta(void);
static unsigned char Hal_Key_GetKey2Sta(void);
static unsigned char Hal_Key_GetKey3Sta(void);
static unsigned char Hal_Key_GetKey4Sta(void);
static unsigned char Hal_Key_GetNetKeySta(void);

unsigned char (*GetKeyState[KEYNUM])(void) = {
    Hal_Key_GetKey1Sta,
    Hal_Key_GetKey2Sta,
    Hal_Key_GetKey3Sta,
    Hal_Key_GetKey4Sta,
    Hal_Key_GetNetKeySta
};

KEY_STEP_TYPEDEF KeyStep[KEYNUM];
unsigned short KeyScanTimer[KEYNUM];
unsigned short KeyPressLongTimer[KEYNUM];
unsigned short KeyContPressTimer[KEYNUM];

KeyEvent_CallBack_t KeyScanCBS;

/**
 * @name    Hal_Key_Init
 * @brief   Initialize the keyboard scanning module.
 * @param   None
 */
void Hal_Key_Init(void)
{
    KeyScanCBS = 0;
    Hal_Key_Config();
}

/**
 * @name    Hal_Key_Config
 * @brief   the hardware abstraction layer functions for keys
 * @param   None
 */
void Hal_Key_Config(void)
{
    unsigned char i;
    
    GPIO_Init(KEY1_PORT, KEY1_PIN, GPIO_Mode_In_PU_IT);
    EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Falling);  

    GPIO_Init(KEY2_PORT, KEY2_PIN, GPIO_Mode_In_PU_IT);       
    EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);  

    GPIO_Init(KEY3_PORT, KEY3_PIN, GPIO_Mode_In_PU_IT);       
    EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Falling);  

    GPIO_Init(KEY4_PORT, KEY4_PIN, GPIO_Mode_In_FL_IT);      // PC1 no pull-up resistance
    EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Falling); 

    for(i=0; i<KEYNUM; i++)
    {
        KeyStep[i] = KEY_STEP_WAIT;
        KeyScanTimer[i] = KEY_SCANTIME;
        KeyPressLongTimer[i] = KEY_PRESS_LONG_TIME;
        KeyContPressTimer[i] = KEY_PRESS_CONTINUE_TIME;
    }
}

/**
 * @name    Hal_Key_KeyScanCBSRegister
 * @brief   KeyScan call-back function register
 * @param   pCBS-> point to incoming call-back function
 */
void Hal_Key_KeyScanCBSRegister(KeyEvent_CallBack_t pCBS)
{
    if(KeyScanCBS == 0)
    {
        KeyScanCBS = pCBS;
    }
}

/**
 * @name    Hal_Key_Pro
 * @brief   KeyScan polling function
 * @param   None
 */
void Hal_Key_Pro(void)
{
    unsigned char i;
    unsigned char KeyState[KEYNUM];
    unsigned char KeyValue;
    
    for(i=0; i<KEYNUM; i++)
    {
        KeyValue = 0;
        
        KeyState[i] = GetKeyState[i]();
        
        switch(KeyStep[i])
        {
            case KEY_STEP_WAIT:
                {
                    if(KeyState[i])
                    {
                        KeyStep[i] = KEY_STEP_CLICK;
                    }
                }
                break;
                
            case KEY_STEP_CLICK:
                {
                    if(KeyState[i])
                    {
                        if(!(--KeyScanTimer[i]))
                        {
                            KeyScanTimer[i] = KEY_SCANTIME;
                            KeyStep[i] = KEY_STEP_LONG_PRESS;
                            KeyValue = (i*5) + 1;
                        }
                    }
                    else
                    {
                        KeyScanTimer[i] = KEY_SCANTIME;
                        KeyStep[i] = KEY_STEP_WAIT;
                    }
                }
                break;
            
            case KEY_STEP_LONG_PRESS:
                {
                    if(KeyState[i])
                    {
                        if(!(--KeyPressLongTimer[i]))
                        {
                            KeyPressLongTimer[i] = KEY_PRESS_LONG_TIME;
                            KeyStep[i] = KEY_STEP_CONTINUOUS_PRESS;
                            KeyValue = (i*5) + 3;
                        }
                    }
                    else
                    {
                        KeyPressLongTimer[i] = KEY_PRESS_LONG_TIME;
                        KeyStep[i] = KEY_STEP_WAIT;
                        KeyValue = (i*5) + 2;
                    }
                }
                break;
            
            case KEY_STEP_CONTINUOUS_PRESS:
                {
                    if(KeyState[i])
                    {
                        if(!(--KeyContPressTimer[i]))
                        {
                            KeyContPressTimer[i] = KEY_PRESS_CONTINUE_TIME;
                            KeyValue = (i*5) + 4;
                        }
                    }
                    else
                    {
                        KeyContPressTimer[i] = KEY_PRESS_CONTINUE_TIME;
                        KeyStep[i] = KEY_STEP_WAIT;
                        KeyValue = (i*5) + 5;
                    }
                }
                break;      
        }
        if(KeyValue)
        {
            KeyScanCBS((KEY_VALUE_TYPEDEF)KeyValue);
        }
    }
}

/*----------------------------------------------------------------------------------------*/
static unsigned char Hal_Key_GetKey1Sta(void)
{
    return (!(GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN)));
}

static unsigned char Hal_Key_GetKey2Sta(void)
{
    return (!(GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN)));
}

static unsigned char Hal_Key_GetKey3Sta(void)
{
    return (!(GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN)));
}

static unsigned char Hal_Key_GetKey4Sta(void)
{
    return (!(GPIO_ReadInputDataBit(KEY4_PORT, KEY4_PIN)));
}

static unsigned char Hal_Key_GetNetKeySta(void)
{
    if((!GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN)) && (!(GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN))))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @name    Hal_Key_GetAllKeyState
 * @brief   get key state
 * @param   None
 */
unsigned char Hal_Key_GetAllKeyState(void)
{
    if((!(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN)))
    || (!(GPIO_ReadInputDataBit(KEY2_PORT,KEY2_PIN)))
    || (!(GPIO_ReadInputDataBit(KEY3_PORT,KEY3_PIN)))
    || (!(GPIO_ReadInputDataBit(KEY4_PORT,KEY4_PIN))))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/*------------------------------- Interrupt Handler ------------------------------------------------*/
INTERRUPT_HANDLER(EXTI1_IRQHandler, 9)
{
    EXTI_ClearITPendingBit(EXTI_IT_Pin1);
}

INTERRUPT_HANDLER(EXTI2_IRQHandler, 10)
{
    EXTI_ClearITPendingBit(EXTI_IT_Pin2);
}

INTERRUPT_HANDLER(EXTI3_IRQHandler, 11)
{
    EXTI_ClearITPendingBit(EXTI_IT_Pin3);
}

INTERRUPT_HANDLER(EXTI4_IRQHandler, 12)
{
    EXTI_ClearITPendingBit(EXTI_IT_Pin4);
}

