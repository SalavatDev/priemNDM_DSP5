/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"



/* USER CODE BEGIN 0 */
#include "i2c.h"
#include "gpio.h"
#include <math.h>
#include <string.h>
#include "tim.h"
#include "spi.h"
#include <math.h>    
#include "stm32l4xx_it.h"
    
extern   uint8_t  nBytesDataPack; 

extern uint8_t  nSendByteNDM; 
extern uint8_t  nSendByteMIR;

extern __IO uint8_t halt;
extern        uint16_t  levelCodAdc ; 
extern uint16_t   levAdcMax; 
extern uint16_t   levAdcMin; 

uint32_t   prdUTCtime = 0;
uint32_t   currentUtcTime;
uint32_t   sumcurrentUtcTime;

extern ftime_t   typeDataTime;
 
extern uint8_t   type_pribora;
extern uint8_t   resultValid[]; 
extern uint8_t   mirParamTRChan[];
extern uint8_t   dataWRflash[];
extern uint8_t   fSinchr;
extern uint16_t  rxNdmCadr[NUM_POINTS_INCADR];

uint8_t resultNdmParam[NUMBYTE_INSEVEN_CHANELMODE+1]={0};
typePhase_TD phase=UNDEF_PHASE;
extern uint16_t dataSS[] ; 
float  cf1=0.0, cf2=0.0; 

uint16_t rxUsartBuf=0;

UART_HandleTypeDef huart3;
const uint8_t verSoftware[3]={0,2,7};
const uint16_t verSoft = 0x0207;

 
 
int ycs[128]=
{
  1,1,-1,-1,1,1,-1,1, //b3
  1,1,1,1,-1,-1,1,1, //cf
  -1,-1,1,-1,1,1,1,1 //f4
};



const uint8_t crctable[256]={
    
	 0  , 94 , 188, 226, 97 , 63 , 221, 131, 194, 156, 126, 32 , 163, 253, 31 , 65,
	 157, 195, 33 , 127, 252, 162, 64 , 30 , 95 , 1  , 227, 189, 62 , 96 , 130, 220,
	 35 , 125, 159, 193, 66 , 28 , 254, 160, 225, 191, 93 , 3  , 128, 222, 60 , 98,
	 190, 224, 2  , 92 , 223, 129, 99 , 61 , 124, 34 , 192, 158, 29 , 67 , 161, 255,
	 70 , 24 , 250, 164, 39 , 121, 155, 197, 132, 218, 56 , 102, 229, 187, 89 , 7,
	 219, 133, 103, 57 , 186, 228, 6  , 88 , 25 , 71 , 165, 251, 120, 38 , 196, 154,
	 101, 59 , 217, 135, 4  , 90 , 184, 230, 167, 249, 27 , 69 , 198, 152, 122, 36,
	 248, 166, 68 , 26 , 153, 199, 37 , 123, 58 , 100, 134, 216, 91 , 5  , 231, 185,
	 140, 210, 48 , 110, 237, 179, 81 , 15 , 78 , 16 , 242, 172, 47 , 113, 147, 205,
	 17 , 79 , 173, 243, 112, 46 , 204, 146, 211, 141, 111, 49 , 178, 236, 14 , 80,
	 175, 241, 19 , 77 , 206, 144, 114, 44 , 109, 51 , 209, 143, 12 , 82 , 176, 238,
	 50 , 108, 142, 208, 83 , 13 , 239, 177, 240, 174, 76 , 18 , 145, 207, 45 , 115,
	 202, 148, 118, 40 , 171, 245, 23 , 73 , 8  , 86 , 180, 234, 105, 55 , 213, 139,
	 87 , 9  , 235, 181, 54 , 104, 138, 212, 149, 203, 41 , 119, 244, 170, 72 , 22,
	 233, 183, 85 , 11 , 136, 214, 52 , 106, 43 , 117, 151, 201, 74 , 20 , 246, 168,
	 116, 42 , 200, 150, 21 , 75 , 169, 247, 182, 232, 10 , 84 , 215, 137, 107, 53
           
 };

/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 172800;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


////////////////////////////////////////////////////
  /*
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
*/
/////////////////////////////////////////////////////  
  
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
     HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
     HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }else if(uartHandle->Instance==USART3)
  {
     __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  
  
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
#define HAL_IWDG_DEFAULT_TIMEOUT            48u

#define IWDG_KEY_RELOAD                 0x0000AAAAu  /*!< IWDG Reload Counter Enable   */
#define IWDG_KEY_ENABLE                 0x0000CCCCu  /*!< IWDG Peripheral Enable       */
#define IWDG_KEY_WRITE_ACCESS_ENABLE    0x00005555u  /*!< IWDG KR Write Access Enable  */
#define IWDG_KEY_WRITE_ACCESS_DISABLE   0x00000000u  /*!< IWDG KR Write Access Disable */

#define __HAL_IWDG_RELOAD_COUNTER(__HANDLE__)       WRITE_REG((__HANDLE__)->Instance->KR, IWDG_KEY_RELOAD)
#define __HAL_IWDG_START(__HANDLE__)   WRITE_REG((__HANDLE__)->Instance->KR, IWDG_KEY_ENABLE)
#define IWDG_ENABLE_WRITE_ACCESS(__HANDLE__)  WRITE_REG((__HANDLE__)->Instance->KR, IWDG_KEY_WRITE_ACCESS_ENABLE)

IWDG_HandleTypeDef hiwdg;
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)
{
  uint32_t tickstart;

  /* Check the IWDG handle allocation */
  if (hiwdg == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_IWDG_ALL_INSTANCE(hiwdg->Instance));
  assert_param(IS_IWDG_PRESCALER(hiwdg->Init.Prescaler));
  assert_param(IS_IWDG_RELOAD(hiwdg->Init.Reload));
  assert_param(IS_IWDG_WINDOW(hiwdg->Init.Window));

  /* Enable IWDG. LSI is turned on automatically */
  __HAL_IWDG_START(hiwdg);

  /* Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers by writing
  0x5555 in KR */
  IWDG_ENABLE_WRITE_ACCESS(hiwdg);

  /* Write to IWDG registers the Prescaler & Reload values to work with */
  hiwdg->Instance->PR = hiwdg->Init.Prescaler;
  hiwdg->Instance->RLR = hiwdg->Init.Reload;

  /* Check pending flag, if previous update not done, return timeout */
  tickstart = HAL_GetTick();

  /* Wait for register to be updated */
  while (hiwdg->Instance->SR != 0x00u)
  {
    if ((HAL_GetTick() - tickstart) > HAL_IWDG_DEFAULT_TIMEOUT)
    {
      return HAL_TIMEOUT;
    }
  }

  /* If window parameter is different than current value, modify window
  register */
  if (hiwdg->Instance->WINR != hiwdg->Init.Window)
  {
    /* Write to IWDG WINR the IWDG_Window value to compare with. In any case,
    even if window feature is disabled, Watchdog will be reloaded by writing
    windows register */
    hiwdg->Instance->WINR = hiwdg->Init.Window;
  }
  else
  {
    /* Reload IWDG counter with value defined in the reload register */
    __HAL_IWDG_RELOAD_COUNTER(hiwdg);
  }

  /* Return function status */
  return HAL_OK;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
  /* Reload IWDG counter with value defined in the reload register */
  __HAL_IWDG_RELOAD_COUNTER(hiwdg);

  /* Return function status */
  return HAL_OK;
}

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/*
void send_byteUART3(uint8_t data){

   while(!(huart3.Instance->ISR & (1<<6)));
    huart3.Instance->TDR = data ;
     huart3.Instance->ICR |= (1<<6);
};
*/
/*
float findSynchrPacket(uint16_t *in){
  
  uint8_t i;
  int centrSign[192];
  int tmpD=0;
  float normSUM=0; 
  int srZn=0;
  float norm=0;
  float skalyrMult=0;
  
  for(i=0;i<192;i++){
    tmpD += in[i];
  }
  srZn = (int) (tmpD * 0.0052083); 
  
  for(i=0;i<192;i++)
  {
    centrSign[i] = (int)srZn -(int)in[i];
    normSUM += (centrSign[i] * centrSign[i]);  
  }
  
  for(i=0;i<192;i++)
  {     
      skalyrMult += centrSign[i] * y[i];        
  }
  
  norm =  sqrt(normSUM);
  if(norm!=0){ 
    return  (skalyrMult /(norm * 13.8564) );  //13.856-192 11.3137-128
  }else{
    return 0.0;
    }

};
 */

long int findPhasePoint(uint16_t *in, uint32_t *isp ){
  
   uint16_t i=0;
  int k=0;
  uint32_t res=0;
  long int corSum = 0;
  int *py = ycs;
  
  while(i < SYNCHR_LEHGTH)
   {  
     k = in[i++]+in[i++]+in[i++]+in[i++];
     k-= (in[i++]+in[i++]+in[i++]+in[i++]);     
     res >>= 1;
     if(k>0)
        res |= 0x00800000;  

     corSum += *py++ * k;     
    
   }
  
  if((res==0xf4cfb3)||(res==0x0b304c))
  {   
      *isp = res;
      
      
  }else  
  {
      *isp = 0;
     
  }
   
  return corSum;

};
 
/*
long int findPhasePoint_inv(uint16_t *in, uint32_t *isp ){
  
   uint16_t i=0;
  int k=0;
  uint32_t res=0;
  long int corSum = 0;
  int *py = ycs;
  
  while(i < SYNCHR_LEHGTH)
   {  
     k -= in[i++]+in[i++]+in[i++]+in[i++];
     k = (in[i++]+in[i++]+in[i++]+in[i++]);     
     res >>= 1;
     if(k>0)
        res |= 0x00800000;  

     corSum += *py++ * k;     
    
   }
  
  if((res==0xf4cfb3)||(res==0x0b304c))
  {   
      *isp = res;
      
      
  }else  
  {
      *isp = 0;
     
  }
   
  return corSum;

};

*/

/*
float coeffCorelytion(uint16_t *in){
  
  uint8_t i;
  int centrSign[8];
  int tmpD=0;
  uint32_t normSUM=0; 
  int srZn=0;
  float norm=0;
  int skalyrMult=0;
  
  for(i=0;i<8;i++){
    tmpD += in[i];
  }
  srZn = (int) (tmpD * 0.125); 
  
  for(i=0;i<8;i++)
  {
    centrSign[i] = (int)srZn -(int)in[i];
    normSUM += (centrSign[i] * centrSign[i]);  
  }
  
  skalyrMult = centrSign[0]+centrSign[1]+centrSign[2]+centrSign[3]-
  centrSign[4]-centrSign[5]-centrSign[6]-centrSign[7] ; 
  
  norm =  sqrt(normSUM);
  if(norm!=0){ 
    return  (skalyrMult /(norm*2.828));
  }else{
    return 0.0;
    }

};
 */
 
static uint8_t getcrc8(const uint8_t *table, uint8_t *pdata, uint8_t nbytes )
{
  uint8_t crc=0;
    while (nbytes-- )
        crc = table[(crc ^ *pdata++) ];
  
    return crc;
}
 


static void inverseToStraight(uint8_t *pdata,const uint8_t nbytes ){
  uint8_t i;
  for(i=0;i<nbytes;i++){
    pdata[i]= ~pdata[i];
  }
}



uint8_t corDataSignal(uint16_t *in){
  
  uint8_t res=0;
  uint8_t i=0;
  int k=0;  
   while(i < NUM_POINTS_INBYTE)
   {
     k = in[i++]+in[i++]+in[i++]+in[i++];
     k-= (in[i++]+in[i++]+in[i++]+in[i++]);     
     res >>= 1;
     if(k>0)
        res |= 0x80;       
     k=0;
   }
 
 return res;
 
};

uint8_t findMaxPoint(long int *inBuf)
{
  uint8_t ires=0;
  long int maxVal=0;
   
  maxVal = *inBuf;  
  for(uint8_t i = 0; i < 8; i++)
  {
    if(maxVal < inBuf[i])
    {
      maxVal = inBuf[i]; 
      ires = i;
    }
    
  }
  
  return ires;
  
}

/*
long int bufCorSum_inv[8];
long int bufCorSum[8];
uint32_t isSP[8];
uint32_t isSP_inv[8];
uint16_t indPoint = 0;
*/

long int cfCorPic[8];
uint32_t spBuf[8];
uint8_t isValidCrc = 0;
uint8_t prd_crc = 0;
uint8_t curent_crc = 0;

void allNDMDataRecive(void)
{
 
 if(fSinchr==3)
 {
    
   uint16_t indPoint = 0; 
   uint8_t iByte=0; 
   uint8_t i; 
   memcpy(rxNdmCadr,dataSS,SYNCHR_LEHGTH*sizeof(uint16_t));
    
    for( i=0; i<8; i++) 
    {    
      cfCorPic[i] = (long int)fabs(findPhasePoint(&rxNdmCadr[i],&spBuf[i]));   
    }
 
    i = findMaxPoint(cfCorPic);    
    if(spBuf[i] == 0xf4cfb3)
    {
      phase = NORMAL_PHASE;
    }else if(spBuf[i] == 0x0b304c)
    {
      phase = INVERSE_PHASE;
    }
    else
    {
      phase = UNDEF_PHASE;
    }
    
    indPoint = i + NUM_POINTS_INBYTE*3; //i + NUM_POINTS_INBYTE*2 - 1;    
      
    GPIOB->BRR = GPIO_PIN_3;
    
    {
        
        while(iByte < (nBytesDataPack+1))
        { 
          resultNdmParam[iByte++]=corDataSignal(&rxNdmCadr[indPoint]); 
          indPoint += NUM_POINTS_INBYTE;            
        }
        
        
        iByte=0;
        if(phase == INVERSE_PHASE)inverseToStraight(&resultNdmParam[0],nBytesDataPack+1);
        
        curent_crc = getcrc8(&crctable[0], &resultNdmParam[0], nBytesDataPack);
        if(curent_crc == resultNdmParam[nBytesDataPack])
          isValidCrc = 1;
        else
          isValidCrc = 0;
        
        
        
    }
      
    cntEnWriteToFlash = 0;
    if(isValidCrc)
    {
              
       GPIOB->ODR ^= GPIO_PIN_2;
       memcpy(&resultValid[0],&resultNdmParam[0],nBytesDataPack);
       memcpy(&dataWRflash[0],&resultNdmParam[0],nBytesDataPack);
       memcpy(&mirParamTRChan[0],&resultNdmParam[0],nBytesDataPack);
              
       if(!halt)
         type_pribora = ((dataWRflash[NUMBYTE_INSEVEN_CHANELMODE-1] & 0xe0)>>5);
       else
         type_pribora = 0xff;
               
               
       GetTimeRTC();               
       currentUtcTime = rtc_code();
       if(!prdUTCtime) 
         sumcurrentUtcTime = 60;
       else
        sumcurrentUtcTime = currentUtcTime - prdUTCtime;
       
       if((curent_crc != prd_crc)||((sumcurrentUtcTime >= 8) && (type_pribora == prd_crc)))
       { 
         //GPIOB->BSRR = GPIO_PIN_4;
         WrToflash();        
         //GPIOB->BRR = GPIO_PIN_4;
       } 
       
 
       prdUTCtime = currentUtcTime;
                 
       if(!halt)
       {
         if((type_pribora == PRIZNAK_NDM_SEVENCHANEL)||(type_pribora == PRIZNAK_NDM_FOURTHCHANEL))
          nSendByteNDM = NUMBYTE_INSEVEN_CHANELMODE;
         else
          nSendByteMIR = NUMBYTE_INSEVEN_CHANELMODE;
       }
       else{
          if((!type_pribora)||(type_pribora == PRIZNAK_NDM_FOURTHCHANEL))
            nSendByteNDM = NUMBYTE_INSEVEN_CHANELMODE/2+1;
          else
            nSendByteMIR = NUMBYTE_INSEVEN_CHANELMODE/2+1;
       }
             
       isValidCrc = 0;    
       
    }
    
    prd_crc = curent_crc;    
    
    if(levAdcMax>=levAdcMin)      
      levelCodAdc = (uint16_t) levAdcMax - levAdcMin ;
    else      
      levelCodAdc = (uint16_t)levAdcMin - levAdcMax;
        
    fSinchr=0;
    memset(dataSS,0,SYNCHR_LEHGTH*sizeof(uint16_t));          
   
    HAL_TIM_Base_Start_IT(&htim6); 

  }
  
     
};
 

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
