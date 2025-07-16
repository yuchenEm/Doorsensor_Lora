/****************************************************
  * @Name	Hal_Flash.c
  * @Brief	
*****************************************************/

#include "stm8l10x.h"
#include "hal_flash.h"

#define BLOCK_OPERATION    127

/*----------------------------------------------------------------------------
@Name		: hal_flash_WriteDat(unsigned char *buf)
@Function	: write data to flash
@Parameter	: buf->point to the buffer to be written
------------------------------------------------------------------------------*/
void hal_flash_WriteDat(unsigned char *buf)   
{
    /* Define flash programming Time*/
    FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);

    FLASH_Unlock(FLASH_MemType_Program);
    /* Wait until Flash Program area unlocked flag is set*/
    while (FLASH_GetFlagStatus(FLASH_FLAG_PUL) == RESET)
    {
        
    }
    /* Unlock flash data eeprom memory */
    FLASH_Unlock(FLASH_MemType_Data);
    /* Wait until Data EEPROM area unlocked flag is set*/
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
    {
        
    }
    /* This function is executed from RAM */
    FLASH_ProgramBlock(BLOCK_OPERATION, FLASH_ProgramMode_Standard,buf);
}

/*----------------------------------------------------------------------------
@Name		: hal_flash_ReadDat(unsigned char *buf,unsigned char len)
@Function	: read data from flash
@Parameter	: buf->point to the buffer to be read
              len->the length of data to be read
------------------------------------------------------------------------------*/
void hal_flash_ReadDat(unsigned char *buf,unsigned char len)
{
    unsigned char i;
    uint32_t  startadd;
    
    startadd = FLASH_START_PHYSICAL_ADDRESS + ((uint16_t)BLOCK_OPERATION * (uint16_t)FLASH_BLOCK_SIZE);
    
    for(i=0;i<len;i++)
    {
        buf[i] = FLASH_ReadByte(startadd++);
    }
}

