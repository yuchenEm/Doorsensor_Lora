/********************************************************
  * @Name   Mid_Protocol.c
  * @Brief  Customized communication protocol between Lora module and Host
  *      Sensor/Remote->Host:   
  *         header:                 FE
  *          datelength:             0x10 (16 byte before checksum)
  *          function code:          1 byte
  *          NodeNo.(CRC):           2 byte
  *          sensor MAC address:     12 byte
  *          sensor type:            1 byte
  *          checksum:               1 byte
  *      Host->Sensor/Remote:
  *          header:                 FE
  *          datelength:             0x10 (16 byte before checksum)
  *          return function code:   1 byte
  *          allocated NodeNo.(CRC): 2 byte
  *          sensor MAC address:     12 byte
  *          sensor type:            1 byte
  *          checksum:               1 byte
  *******************************************************/

#include "stm8l10x.h"
#include "mid_protocal.h"
#include "mid_sx1278.h"
#include "queue.h"

volatile Queue64 LoraRxMsg;

unsigned char LoraRxBuff[32];

ProtocalEvent_CallBack_t pEventCBS;

/*********************************************************************
 * Name         : mid_Protocal_Init
 * Function     : protocol inital
 * Parameter    : None
**********************************************************************/
void mid_Protocal_Init(void)
{
    QueueEmpty(LoraRxMsg);
}

/*********************************************************************
 * Name         : mid_Protocal_CBSRegister
 * Function     : call-back function register
 * Parameter    : pCBS->point to the incoming call-back function
**********************************************************************/
void mid_Protocal_CBSRegister(ProtocalEvent_CallBack_t pCBS)
{
    if(pEventCBS == 0)
    {
        pEventCBS = pCBS;
    }
}

/*********************************************************************
 * Name         : mid_Protocal_Pro
 * Function     : polling function
 * Parameter    : None
**********************************************************************/
void mid_Protocal_Pro(void)
{
    /***************************************************
    /    RxFrame:
    /        head:                   FE
    /        datelength:             0x10 (16 byte before checksum)
    /        return function code:   1 byte
    /        allocated NodeNo.(CRC): 2 byte
    /        sensor MAC address:     12 byte
    /        sensor type:            1 byte
    /
    /        checksum:               1 byte
    ****************************************************/
    static unsigned char DataLength = 0;
    static unsigned char DelayTime = 0;
    unsigned char i, dat, CheckSum;
    
     if(DataLength == 0)
     {
        if(QueueDataLen(LoraRxMsg))
        {
            QueueDataOut(LoraRxMsg, &dat);
            
            if(dat == 0xFE)
            {
                QueueDataOut(LoraRxMsg, &DataLength);
                DelayTime = 0;
            }
        }
     }
     
     if(DataLength > 0)
     {
        if(QueueDataLen(LoraRxMsg) > DataLength) // receive 1 complete dataframe from Host
        {
            if(DataLength > 99) // invalid datalength
            {
                QueueEmpty(LoraRxMsg);
                return;
            }
            
            CheckSum = 0;
            
            for(i=0; i<DataLength; i++)
            {
                QueueDataOut(LoraRxMsg, &LoraRxBuff[i]);
                
                CheckSum += LoraRxBuff[i];
            }
            
            QueueDataOut(LoraRxMsg, &dat);  // queueout checksum
            
            if(dat == CheckSum)
            {
                pEventCBS(&LoraRxBuff[0]);
            }
            DataLength = 0;
        }
        else
        {
            DelayTime++;
            if(DelayTime >= 100)
            {
                DelayTime = 0;
                DataLength = 0;
            }
        }
     }
}

/*********************************************************************
 * Name         : mid_lora_TxDataPackage
 * Function     : prepare the data package to send
 * Parameter    : cmd-> command code
 *                pNode-> point to the Node index(use CRC value)
**********************************************************************/
void mid_lora_TxDataPackage(en_lora_eventTypedef cmd, unsigned char *pNode)
{
    unsigned char i;
    unsigned char txDataTempBuff[20]; 
    unsigned char idx;
    unsigned char uid[12];
    unsigned char len, checkSum;
    
    txDataTempBuff[0] = 0xFE;   
    idx = 1;
    
    if(cmd == LORA_COM_APPLY_NET)
    {
        txDataTempBuff[idx++] = 16;	                    // length
        txDataTempBuff[idx++] = LORA_COM_APPLY_NET;	    // function code
        txDataTempBuff[idx++] = pNode[0];		        // CRC low byte
        txDataTempBuff[idx++] = pNode[1];		        // CRC high byte
      
        for(i=0; i<12; i++)
        {
            uid[i] = *((__IO uint8_t *)(0x4925 + i));
            txDataTempBuff[idx++] = uid[i];
        }
        txDataTempBuff[idx++] = 2;	    //sensor type，2-->remote           
    }
    else if(cmd <= LORA_COM_DOORCLOSE)
    {
        txDataTempBuff[idx++] = 3;	        //length
        txDataTempBuff[idx++] = cmd;		//function code
        txDataTempBuff[idx++] = pNode[0];	//CRC high byte
        txDataTempBuff[idx++] = pNode[1];	//CRC low byte      
    }
    
    if(idx > 3)
    {    
        len = idx - 2;            //exclude head,length
        checkSum = 0;
        for(i=0; i<len; i++)
        {
            checkSum += txDataTempBuff[2+i];	// calculate checksum
        }
        txDataTempBuff[idx++] = checkSum;         //CRC sumcheck
        
        mid_Sx1278_LoRaSentBuffer(txDataTempBuff, idx);
    }	
}
