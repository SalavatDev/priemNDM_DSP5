/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "i2c.h"
#include <string.h>
 
 uint16_t cntPack=0;
 uint16_t currDevelopment=0;    //tekushaya narabotka
 uint16_t totalDevelopment=0;   //obshaya narabotka
 uint16_t  findEndStr=0;  
  
 
 
 extern   __IO uint8_t   halt;
 extern        uint8_t   nBytesDataPack; 
 extern   __IO uint8_t   validCRC ; 
 
 extern   __IO uint32_t  currAdrFlash ;
 extern        uint16_t  levelCodAdc ; 
 extern        uint8_t   dataWRflash[];
 extern        uint8_t   fSinchr;
 extern        uint32_t  startDataFlash;
 extern        uint32_t  endDataFlash;
 extern        ftime_t   typeDataTime;
 extern  const uint16_t verSoft;

 
 

 tdataCorPriemn    flashData;  
 
 uint8_t flashOrAdc=1;//0-adc, 1-flash
     
__IO uint32_t currAdrFlash = STARTADRFLASH; //текущий адрес флешки
//uint16_t  numWrAdrFlash = 1; 

//extern  __IO uint8_t tarir ; 




/* USER CODE END 0 */
 
SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
 
    
  /* USER CODE BEGIN SPI1_MspInit 1 */
 // __HAL_SPI_ENABLE(&hspi1); 
  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */
// __HAL_SPI_DISABLE(&hspi1);
  /* USER CODE END SPI1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
static void delay(__IO uint32_t wait){	
  while(wait--){};
};


extern uint16_t   levAdcMax; 
extern uint16_t   levAdcMin; 

 //READ EXTERNAL ADC AD7980BRM
uint16_t rdDatFromAD7980(){ 
      
        uint16_t rslt=0;
        //uint16_t timOut=200;
        GPIOC->BRR = GPIO_PIN_4;   //  CS_ON
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
        
           GPIOC->BSRR = GPIO_PIN_4;   //  CS_ON
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");   //  asm("nop");    
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");   //  asm("nop");    // Задержка 0.7мкс
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop");  asm("nop");asm("nop");
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); // asm("nop");asm("nop");          
           GPIOC->BRR = GPIO_PIN_4; //      CS_OFF       
            
           
        //считываем полученные данные
          while (!(SPI1->SR & SPI_SR_TXE));
	  SPI1->DR = 0xFFFF;           
          while( SPI1->SR & SPI_SR_BSY);       
        //считываем полученные данные
	  rslt = (uint16_t) (SPI1->DR); 
          

       GPIOC->BSRR = GPIO_PIN_4;   
       
       if(fSinchr){
         if(levAdcMax < rslt){
          levAdcMax = rslt;
        }else
        if(levAdcMin > rslt){
          levAdcMin = rslt;
         }
        }else{
          levAdcMax = levAdcMin = rslt;
        }
    
     
    return  rslt;
} 

//***********************SPI1 LOW LEVEL Functions*********************//


uint8_t sendByte(uint8_t data)
{
  uint8_t result,i;
  i=8;                    
   
   while(i--){
          
    result <<= 1;
    if(data & (0x80)) {
      GPIOA->BSRR = GPIO_PIN_7;    
    }else {
      GPIOA->BRR= GPIO_PIN_7;
    }     
     GPIOA->BRR=GPIO_PIN_5;
     delay(10);
     GPIOA->BSRR=GPIO_PIN_5;     
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){ 
      result |= 1; }
    delay(1);
    data <<= 1;
		
  }  
  return result;
  
}

 

uint8_t readData8(uint8_t address)
{   
  
    uint8_t tempByte;
   
    sendByte(address);
    tempByte=sendByte(0xff);
    // GPIOA->BSRR = GPIO_PIN_7;    
    return tempByte;
    
};

void writeCmnd(uint8_t cmnd ){
   chipL;                  
  sendByte( cmnd );
  chipH;    
}

void readCmnd( uint8_t* tx, uint8_t* rx, uint16_t len ){   
  uint8_t i=0;
   chipL; 
  for(i=0;i<len;i++){
    rx[i] = sendByte(tx[i]);       
  }  
  chipH;     
}

void readFSR(void){   
  uint8_t status=0 ;
  uint8_t timout=100 ;
 chipL;                       
  do{ 
      status = readData8(0x70); 
    }while((!(status & (1<<7)))&&(timout--));  
 
 chipH;   
}

void readSR(void){   
  uint8_t status=0; 
  uint8_t timout=100 ;
   chipL;   
  do{ 
    status = readData8(0x05); 
  } while ((status & 0x01)&&(timout--)); 
  chipH; 
}

void clearFSR(void){     
  chipL;
    sendByte(0x50); 
    chipH; 
}

//******************************************************************//



//*******************USART RS232 LOW LEVEL Functions**************//
 
void send_byteUART(uint16_t data){

   while(!(huart2.Instance->ISR & (1<<6)));
    huart2.Instance->TDR = data ;
     huart2.Instance->ICR |= (1<<6);
};


uint16_t csDataflash ;
void UARTSendData(const uint8_t *in, uint32_t len,uint32_t deltime){
  
  uint32_t i, c;
  uint16_t tmptx;
 
  c=deltime;    //delay during transmit data (in mcs)
 
  for(i=0;i<len;i++){
    
    tmptx = 0x0100 | in[i];
    csDataflash += in[i];
    send_byteUART(tmptx);
    while(c--)asm("nop");
     c=deltime; 
     
  }
  
     
  
}
 
//******************************************************************//




//******************************************************************//
//*************READ/WRITE EXTERNAL FLASH N25Q512A13G1240E***********//
//******************************************************************//


uint32_t rdNonValConfReg(void){   
  
  uint32_t status=0,tx=0xFFFFffB5;
 
  chipL;                  
   
      HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)&tx,(uint8_t*)&status,3,50); 
 
  chipH; 
   return status;
}

 
uint16_t rdExtAdrReg(void){   
  
 uint8_t status=0;
 
  chipL;                  
  
   sendByte(RDEXTADRREG);
   status = sendByte(0xFF);
      
 
  chipH;
  
   return status;
   
}

 
void wrExtAdrReg(uint8_t adrExt){   
  
 chipL;                  
     
   sendByte(WREXTADRREG);
   sendByte(adrExt);

  chipH; 
   
}


uint8_t infoDataFlashTx[256]={
0,3
};

const uint8_t stopTxDataFlash[5]={
  0xEF,0xEF,0xEF,0xEF,0xEF
    
};


static void swapBytes(uint32_t *inD){
  uint32_t tmp;
  uint8_t *ptr;
  
  tmp = 0;
  ptr = (uint8_t*)inD;
  tmp =  (uint32_t)((*ptr) << 16); ptr++;
  tmp |=  (uint32_t)((*ptr) << 8); ptr++;
  tmp |=  (uint32_t) (*ptr); 
  
  *inD = tmp;
  
}



static void txFlashData(uint32_t startAdrRdFlash, uint32_t endAdrRdFlash){
  
  uint32_t i = startAdrRdFlash;
  uint8_t tmpRDFlashBuff[SIZEFORMATFLASH];
 
  
  while((i < NBYTESFLASH) && (i < endAdrRdFlash)){
        
    flashRD(i,(uint8_t *)&tmpRDFlashBuff[0],SIZEFORMATFLASH );  
 
    //////////////////////////////   
    uint8_t tmp[2] = {VER_RDFLASH_FORMAT, ID_DEV};     
    UARTSendData(tmp, 2, 1900); 
   
    /////////////////////////////             
    UARTSendData(tmpRDFlashBuff, SIZEFORMATFLASH, 1900);     
    //cnttxbytes += SIZEFORMATFLASH; //<-----------------
     
    i += SIZEFORMATFLASH; 
    HAL_IWDG_Refresh(&hiwdg);
     
  }
      
}
 
void readDataFromFlash(){
  
 
   uint32_t i=0, tmpInd=0; 
   
   uint32_t startFlashAdr,endFlashAdr;
   uint32_t  cmprTimeFlash;
 
   HAL_IWDG_Refresh(&hiwdg);
   
   memset(infoDataFlashTx,0,256);
   
   startFlashAdr = 0;
   endFlashAdr = 0;
  
   switchToSoftSPI();
   
   infoDataFlashTx[1] = 1; 
   infoDataFlashTx[6] = (uint8_t)(currAdrFlash >>24);
   infoDataFlashTx[7] = (uint8_t)(currAdrFlash >> 16);  
   infoDataFlashTx[8] = (uint8_t)(currAdrFlash >> 8);  
   infoDataFlashTx[9] = (uint8_t)currAdrFlash;
 
   cmprTimeFlash = 0x00ffffff;      
   infoDataFlashTx[3] = (uint8_t)(cmprTimeFlash >> 16);
   infoDataFlashTx[4] = (uint8_t)(cmprTimeFlash >> 8); 
   infoDataFlashTx[5] = (uint8_t)(cmprTimeFlash );        
   HAL_Delay(50);   
   UARTSendData(infoDataFlashTx, 256, 1900);          
   tmpInd = currAdrFlash;  
 
  
   
   {
     
     uint8_t tmp = 0xAA;   
     UARTSendData(&tmp, 1, 1900);

   } 
  
   
   csDataflash = 0;
   i = STARTADRFLASH;
   
   if((!startDataFlash) && (!endDataFlash) ){
            
             
      txFlashData(STARTADRFLASH, NBYTESFLASH); 
     
   
       
   }else if((startDataFlash == 0x00e6e6e6) && (endDataFlash == 0x00e6e6e6)){
     
      
      txFlashData(STARTADRFLASH, tmpInd);
     
   }else if ((startDataFlash) && (endDataFlash)){
     
           
       swapBytes(&startDataFlash);
       swapBytes(&endDataFlash);
       
           if(startDataFlash>endDataFlash) {
               
              cmprTimeFlash = startDataFlash;
              startDataFlash=endDataFlash;
              endDataFlash = cmprTimeFlash;
              cmprTimeFlash=0;
            }
           
  cmprTimeFlash=0; 
  flashRD(i+5,(uint8_t *)&cmprTimeFlash, 3); 

  if(cmprTimeFlash > startDataFlash){
      
    startFlashAdr = i;
    
  }else{
    
      while(i < NBYTESFLASH)
      {
        cmprTimeFlash=0; 
        flashRD(i+5,(uint8_t *)&cmprTimeFlash, 3); 
        
        if((cmprTimeFlash == startDataFlash) && (startFlashAdr == 0xffffffff)){
          startFlashAdr = i;   
          break;
        }
        
        i += SIZEFORMATFLASH;
      }
  }
 
  if(startFlashAdr == 0xffffffff){return;}
  
  while(i < NBYTESFLASH)
   {
      cmprTimeFlash=0; 
      flashRD(i+5,(uint8_t *)&cmprTimeFlash, 3); 
      
    if(startDataFlash == endDataFlash){
      
      if((cmprTimeFlash != endDataFlash)&&(startFlashAdr != 0xffffffff)){
        endFlashAdr = i;
        break;
      }
        
    }else{
      
      if((cmprTimeFlash > endDataFlash)&&(startFlashAdr != 0xffffffff)){
        endFlashAdr = i;
        break;
      }
        
    }
      
      i += SIZEFORMATFLASH;
    
  }
  
  if(endFlashAdr != 0xffffffff )
   {      
      txFlashData(startFlashAdr, endFlashAdr);        
   } 
  
        
  
   
}

   HAL_Delay(1);
  {    
    uint8_t txtmp[3] = {0, (csDataflash >> 8), (csDataflash & 0x00ff)}; 
    UARTSendData(txtmp, 3, 1900); 
    UARTSendData((const uint8_t*)stopTxDataFlash, 5, 1900);        
  }   
      
  switchToHardSPI();
 
   
}

uint16_t levEreaseFlash; //procent stiraniya flash
extern uint8_t isByteRecived;
extern   uint16_t rxUsartBuf;
void clearFlashFull(){
  
  //tAdrToWrite tmpADRflash;
  uint32_t i=0;   
  uint8_t timOUT = 0;
  tdataCorPriemn tmpWrFlash;
  
  levEreaseFlash=0;
  
  //to do code her...
  timOUT = 15; isByteRecived = 0;
  while((!isByteRecived)&&(timOUT--))
  {
    HAL_Delay(100);
  }
  
  if(rxUsartBuf == 0x00ef)
    return;
  
  switchToSoftSPI();
  
  while(i < NBYTESFLASH ){
    
   HAL_IWDG_Refresh(&hiwdg); 
   
   flashRD((uint32_t)i+INDENDSTRFLASH,(uint8_t *)&findEndStr,sizeof(uint16_t));
   if(findEndStr != 0xffff){
     ereaseFlash(i,THRSUBSECCLR);
     //HAL_Delay(350);
     i += SUBSECSIZE;
     
   }else{
     
     i += SIZEFORMATFLASH;
     
   }
 
   if(levEreaseFlash <= 0x0164)
    levEreaseFlash = (0x0100 | (uint16_t)((i * 100)/NBYTESFLASH));  
    
  }
  
    levEreaseFlash = (0x0100 | (uint16_t)100);
    HAL_Delay(1000);   
      //
  currAdrFlash = STARTADRFLASH;     
  tmpWrFlash.NaraBTotal = totalDevelopment;
  tmpWrFlash.NarabTek = currDevelopment;
  tmpWrFlash.regimChanel = (uint16_t)halt;  
  tmpWrFlash.endString = ENDSTRINGFLASH;   
  wrFlash(STARTADRFLASH,(uint8_t*)&tmpWrFlash,SIZEFORMATFLASH);  
  writeCmnd(WRITEDIS); //write disable 
    
 
   switchToHardSPI();
   
 
} 

void wrFlash(uint32_t adr, uint8_t *dIN,const uint32_t nbytes)   
{
 uint8_t adrBuf[5];   
  
  uint8_t i;
 
 adrBuf[0]=THRPAGEPROG; 
 adrBuf[1]=(uint8_t)((adr & 0x00ff0000) >> 16);
 adrBuf[2]=(uint8_t)((adr & 0x0000ff00) >> 8); 
 adrBuf[3]=(uint8_t)(adr & 0x000000ff); 
 
   readFSR(); 
   writeCmnd(WRITEEN); 
   readFSR();
   
  chipL;
   for(i=0;i<4;i++){
    sendByte(adrBuf[i]); }
  
   for(i=0;i<nbytes;i++){
    sendByte(dIN[i]); }
   
  chipH;
  
   HAL_Delay(50);
   readFSR();
   
}
 


uint16_t levCodADC=0;
HAL_StatusTypeDef status_I2C=HAL_OK; 
extern uint8_t   type_pribora;
extern uint32_t   sumcurrentUtcTime;
uint8_t enWrDataToFlash = 0;
/////////////////////107MS//////////////////////////
void WrToflash(){

 // HAL_TIM_Base_Stop_IT(&htim6);
  
    
   currDevelopment += sumcurrentUtcTime;
   
  if(currDevelopment >= 1800){    
    ++totalDevelopment;   
    currDevelopment = 0;
  }
   
  memset((void*)&flashData,0,sizeof(tdataCorPriemn)); //clear result 
  
  switchToSoftSPI();
 
/***********Write time,LevelADCCorRECIVER*************/
        
  if(status_I2C != 0){   
    HAL_I2C_MspDeInit(&hi2c1);
    MX_I2C1_Init();
  }
  
     flashData.dataTime[0]=dataRTC[0];  flashData.dataTime[1] = dataRTC[1]; 
     flashData.dataTime[2]=dataRTC[2];  flashData.dataTime[3] = dataRTC[3];
     flashData.dataTime[4]=dataRTC[4];  flashData.dataTime[5] = dataRTC[5];
   
    flashData.levEMCorRec = levCodADC ;       //LevelADCCorRECIVER, 
    flashData.levTRANSFCorRec = levCodADC;   //LevelADCCorRECIVER,  
    
    
 /*****************ID Packet, Soft Version*******************/
    
    flashData.idPack = ID_DEV;
    flashData.verProg = verSoft;
    
 /***************Corelytion Reciver from NDM OR MIR****************/   
 
         
    if((type_pribora == PRIZNAK_NDM_SEVENCHANEL)||(type_pribora == PRIZNAK_NDM_FOURTHCHANEL))
    { 
      memcpy(&flashData.dtNDM.ZenitUpit,&dataWRflash[0],nBytesDataPack);    
          
    }else if((type_pribora == PRIZNAK_MIK_ZTK) || (type_pribora == PRIZNAK_MIR_GEOMASH))
    {
      memcpy(&flashData.dtMIR.zenitIKUpit,&dataWRflash[0],nBytesDataPack);
          
    }        
 
    memcpy( &dataWRflash[0],0,nBytesDataPack); 
 
/*******************************level signal***************************/    
 
      flashData.levEMCorRec = levelCodAdc / 2 ; 
    
/*********************************************************************/   
     flashData.regimChanel = (uint16_t)halt;  
     flashData.numPackage = cntPack;    
     cntPack++;     
     flashData.NarabTek = currDevelopment;     
     flashData.NaraBTotal = totalDevelopment;  
     
     flashData.endString = 0x0201; 
 
   
 /******************Write To Flash 105MS***********************/   
     
     flashRD((uint32_t)currAdrFlash+INDENDSTRFLASH,(uint8_t *)&findEndStr,sizeof(uint16_t));
     
     if(findEndStr != 0xffff){
        readFSR(); 
        writeCmnd(WRITEEN);
        ereaseFlash(currAdrFlash,THRSUBSECCLR);
     }  
       
 
      
     if(currAdrFlash < NBYTESFLASH){    
      
       wrFlash(currAdrFlash,(uint8_t*)&flashData ,SIZEFORMATFLASH); 
       currAdrFlash += SIZEFORMATFLASH ;    
       
     }else{
             
      currAdrFlash = STARTADRFLASH;        
      readFSR(); 
      writeCmnd(WRITEEN);
      ereaseFlash(STARTADRFLASH,THRSUBSECCLR);
      
     }
   
 
    
    HAL_Delay(1);
    writeCmnd(WRITEDIS);        //write disable 
 
  /*******************************************************/  
 
  
    switchToHardSPI();
 
   
  //  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);   
  //  HAL_TIM_Base_Start_IT(&htim6);
      
  
}
  

void ereaseFlash(uint32_t adr, uint8_t cmnd)   
{
  
 uint8_t dataBuf[5]={0,0,0,0};  
  uint8_t i;
 dataBuf[0]= cmnd;
 dataBuf[1]=(uint8_t)((adr & 0x00ff0000) >> 16);
 dataBuf[2]=(uint8_t)((adr & 0x0000ff00) >> 8); 
 dataBuf[3]=(uint8_t)(adr & 0x000000ff);
 
  readFSR(); 
  writeCmnd(WRITEEN);
  readFSR();
  
  chipL;
   for(i=0;i<4;i++){
     sendByte(dataBuf[i]); }      
  chipH;
  
   HAL_Delay(300); 
  readFSR();
 
 
 } 
 
 
 
  
void flashRD(uint32_t adr, uint8_t *dOut,const uint32_t nbytes)   
{
  uint8_t   adrBuf[5]; 
  uint8_t ind;    
  
  adrBuf[0] = THRMODERDFLASH;
  adrBuf[1]=(uint8_t)((adr & 0x00ff0000) >> 16);
  adrBuf[2]=(uint8_t)((adr & 0x0000ff00) >> 8); 
  adrBuf[3]=(uint8_t)(adr & 0x000000ff);  
 
  
  chipL;  
 
  for(ind=0;ind<4;ind++){
  sendByte(adrBuf[ind]); }
  
 
   for(ind=0;ind<nbytes;ind++){
    dOut[ind]=sendByte(0xFF); }
 
 chipH;   

}
 
/*
int compareTime(ftime_t realTime,ftime_t lastFlashTime)
{
  
   if(realTime.year > lastFlashTime.year)
   {        
      return 31556926;
   
   }else if(realTime.year==lastFlashTime.year)
   {
     
       if(realTime.month > lastFlashTime.month)
       {
         
         return 2629743 ;
         
       }else if(realTime.month == lastFlashTime.month)
       {
         
            if(realTime.day > lastFlashTime.day)
            {
              
              return 86400;  
              
            }else if(realTime.day == lastFlashTime.day)
            {
              
                if(realTime.hour > lastFlashTime.hour)
                {
                  
                  return 3600; 
                  
                }else if(realTime.hour == lastFlashTime.hour)
                {
                   if(realTime.minute > lastFlashTime.minute)
                   {
                     if((realTime.minute - lastFlashTime.minute)==1)
                     {
                       return (realTime.second+(60-lastFlashTime.second));
                     }else
                     {
                       return ((realTime.minute - lastFlashTime.minute)*60);
                     }
                     
                   }else if(realTime.minute == lastFlashTime.minute)
                   {
                     
                     if(realTime.second >= lastFlashTime.second)
                       
                       return (realTime.second - lastFlashTime.second);
                     
                   }
                  
                }
              
            }         
         
       }
     
     
   }
    

   return -1;
   
};
*/


void FlashInit(void){
  
 
 switchToSoftSPI();
 uint32_t indAdr=0, adrSubSecSize, limSubSecSize ;
 
 tdataCorPriemn tmpRdParamFlash;
   
  readFSR(); 
  writeCmnd(WRITEEN);      
  writeCmnd(THRRBYTEMODE); 
  readFSR();
  writeCmnd(WRITEEN);
  wrExtAdrReg(0);  
  HAL_Delay(1);  
 //
    
    do
    {
          
      flashRD ((uint32_t)(SUBSECSIZE*indAdr)+INDENDSTRFLASH,(uint8_t *)&findEndStr,sizeof(uint16_t));
      indAdr++;
          
    }while((findEndStr != 0xffff) && (indAdr < CNTSUBSECINFLASH));
 
    indAdr--;  
    
    if(indAdr)
    {
            
        limSubSecSize = (indAdr + 1) * SUBSECSIZE;
        adrSubSecSize = indAdr*SUBSECSIZE + INDENDSTRFLASH;
        
        do
        {
   
          flashRD ((uint32_t)adrSubSecSize,(uint8_t *)&findEndStr,sizeof(uint16_t));
          adrSubSecSize += SIZEFORMATFLASH;
                  
        }while((findEndStr != 0xffff) && (adrSubSecSize < limSubSecSize));
              
        
         
        if(adrSubSecSize >= limSubSecSize)
        {   
          
          flashRD ( (NBYTESFLASH + 1) - SIZEFORMATFLASH , (uint8_t *)&tmpRdParamFlash, SIZEFORMATFLASH);
          readFSR(); 
          writeCmnd(WRITEEN);
          ereaseFlash(STARTADRFLASH,THRSUBSECCLR);          
          currAdrFlash = STARTADRFLASH;    
          totalDevelopment = tmpRdParamFlash.NaraBTotal;   
          currDevelopment = tmpRdParamFlash.NarabTek;
          halt = (uint8_t)tmpRdParamFlash.regimChanel;
          
        }else{
          
          currAdrFlash = adrSubSecSize-2*SIZEFORMATFLASH+2;
          flashRD ( currAdrFlash - SIZEFORMATFLASH , (uint8_t *)&tmpRdParamFlash, SIZEFORMATFLASH);
          totalDevelopment = tmpRdParamFlash.NaraBTotal;   
          currDevelopment = tmpRdParamFlash.NarabTek;
          halt = (uint8_t)tmpRdParamFlash.regimChanel;
          
        }  
            
    }else
    {
      
      currAdrFlash = STARTADRFLASH;
      halt = SEVEN_CHANEL_MODE;
      currDevelopment = 0;
      totalDevelopment = 0;
      
    }
 
    
    writeCmnd(WRITEDIS);
    switchToHardSPI(); 
     
 };
  
 

void switchToSoftSPI(){
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  SPI1->CR1 &= ~SPI_CR1_SPE; 
  __HAL_RCC_SPI1_CLK_DISABLE(); 
  
  
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
   
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  
}

void switchToHardSPI(){
  
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI1->CR1 &= ~SPI_CR1_SPE; 
   __HAL_RCC_SPI1_CLK_ENABLE();  
    //SPI1 GPIO Configuration    
  //  PA5     ------> SPI1_SCK
    //PA6     ------> SPI1_MISO
   // PA7     ------> SPI1_MOSI 
   // 
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    SPI1->CR1 |= SPI_CR1_SPE; 
    
};


 

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
