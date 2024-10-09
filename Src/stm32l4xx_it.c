/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"

/* USER CODE BEGIN 0 */
  
#include "spi.h"
#include "usart.h"
#include "i2c.h"
#include <string.h>
#include <math.h>

extern   uint16_t rxUsartBuf; 
extern   __IO uint8_t halt;   
extern   tdataCorPriemn   flashData;

extern uint16_t levEreaseFlash;

extern const uint8_t verSoftware[3];
extern  ftime_t   typeDataTime;
 
uint8_t  nBytesDataPack = NUMBYTE_INSEVEN_CHANELMODE;
uint16_t  nBytesInCadr = NUM_POINTS_INHALF;
uint8_t   resultValid[NUMBYTE_INSEVEN_CHANELMODE];
uint8_t   mirParamTRChan[NUMBYTE_INSEVEN_CHANELMODE];
uint8_t   dataWRflash[NUMBYTE_INSEVEN_CHANELMODE];


taskTARIR taskTarir=noTASK;

uint16_t data1[NPOINTS]; 
 

uint8_t  nSendByteNDM = 0; 
uint8_t  nSendByteMIR = 0; 
uint64_t dataUsart=0;
 
 
 
uint8_t *rdParamUart[255]={
   
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,   
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   (uint8_t*)&mirParamTRChan[0],(uint8_t*)&mirParamTRChan[1], //90 | 91
   (uint8_t*)&mirParamTRChan[2],(uint8_t*)&mirParamTRChan[3], //92 | 93
   (uint8_t*)&mirParamTRChan[4],(uint8_t*)&mirParamTRChan[5], //94 | 95
   (uint8_t*)&mirParamTRChan[6],(uint8_t*)&mirParamTRChan[7], //96 | 97
   (uint8_t*)&mirParamTRChan[8],(uint8_t*)&mirParamTRChan[9], //98 | 99
   (uint8_t*)&mirParamTRChan[10],(uint8_t*)&mirParamTRChan[11], //9a | 9b
   (uint8_t*)&mirParamTRChan[12],(uint8_t*)&mirParamTRChan[13], //9c | 9d
   0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 (uint8_t*)&resultValid[0],(uint8_t*)&resultValid[1],//0xd0|0xd1
 (uint8_t*)&resultValid[2],(uint8_t*)&resultValid[3],//0xd2|0xd3
 (uint8_t*)&resultValid[4],(uint8_t*)&resultValid[5],//0xd4|0xd5
 (uint8_t*)&resultValid[6],(uint8_t*)&resultValid[7],//0xd6|0xd7
 (uint8_t*)&resultValid[8],(uint8_t*)&resultValid[9],//0xd8|0xd9
 (uint8_t*)&resultValid[10],(uint8_t*)&resultValid[11],//0xdA|0xdB
 0,0,// 0xdC|0xdd
 (uint8_t*)&resultValid[12],(uint8_t*)&resultValid[13],//0xdE|0xdf
 0
 }; 
 
uint16_t   levelCodAdc=0; 
uint16_t   levAdcMax = 27000, levAdcMin = 27000; 
 
 /*

uint8_t    c=0, k=0;
uint16_t   levelCodAdc=0; 
int8_t     fSign=0;
uint32_t   cntPic=0; 
uint8_t    resultManch[17];

uint32_t   sinchrDat=0;
__IO uint8_t   validCRC = 0;

*/
 
uint8_t    fSinchr = 0;


uint32_t  startDataFlash;
uint32_t  endDataFlash;
uint8_t   enFillTimeFlash ;
 

 
uint32_t  utcTime=0;
uint32_t  utcTimeCmpr=0;
 
uint8_t   tmpTimeRdBuf[8];

uint16_t   tmp_txByte=0;
 
uint8_t   type_pribora=0;

/****************var, td, correc*********************/

uint16_t  rxNdmCadr[NUM_POINTS_INCADR]={0};
uint16_t indPConRdOutData=0; 
uint8_t cntBitData=0;
uint8_t cntByteData=0;
uint16_t indRecWordNdm=0;
 
uint8_t dtByte=0;
uint8_t isByteRecived = 0;
uint16_t dataSS[SYNCHR_LEHGTH]={0}; 
uint8_t  cntEnWriteToFlash=0; 
uint8_t timOutTxId = 0;
 
uint8_t findSSinhr(uint16_t *in)
{
  uint16_t i=0;
  int k=0;
  uint32_t res=0;
 
  while(i < SYNCHR_LEHGTH)
   {
    
     k = in[i++]+in[i++]+in[i++]+in[i++];
     k-= (in[i++]+in[i++]+in[i++]+in[i++]);     
     res >>= 1;
     if(k>0)
        res |= 0x00800000;       
     k=0;
   }
  
  if((res==0xf4cfb3)||(res==0x0b304c))
  {     
      return 1;      
  }
  
  return 0;
    
}
 

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
   
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}
 
/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

   rxUsartBuf = (uint16_t) USART2->RDR & 0x00ff;   
   isByteRecived = 1;
   dataUsart <<= 8;
   dataUsart |= (uint64_t)rxUsartBuf;
   
  if((taskTarir == clrFLASH)&&(rxUsartBuf == 0x00cb))
  {
    
    HAL_UART_Transmit(&huart2,(uint8_t*)&levEreaseFlash,1,HAL_MAX_DELAY);
    return ;
  }
  
  if(taskTarir!=noTASK)return;
    
 
  
  if(((TarirCMND)dataUsart & 0x000000000000ffff) == STCMNDRDFLASH){
                    
          taskTarir = rdFLASH;
          dataUsart=0;    
          rxUsartBuf |= 0x0100;
          timOutTxId = 50;
          while(timOutTxId--);
          HAL_UART_Transmit(&huart2,(uint8_t*)&rxUsartBuf,1,HAL_MAX_DELAY);    
          
        return;
  }
  
   switch((TarirCMND)dataUsart & 0x0000ffffffffffff){     

 
       case CLEAR_FLASH:        
         taskTarir = clrFLASH;        
         dataUsart=0;    
         rxUsartBuf |= 0x0100;         
         timOutTxId = 50;
          while(timOutTxId--);
         HAL_UART_Transmit(&huart2,(uint8_t*)&rxUsartBuf,1,HAL_MAX_DELAY);          
       break;
  
       case RDTIME:    
          taskTarir = rdTIME;
          dataUsart=0;    
          rxUsartBuf |= 0x0100;
          timOutTxId = 50;
          while(timOutTxId--);
          HAL_UART_Transmit(&huart2,(uint8_t*)&rxUsartBuf,1,HAL_MAX_DELAY);           
        break;
  
     case SET_TIME: 
     case SET_TIME_ALL: 
       taskTarir = setTIME;
       dataUsart=0;      
       rxUsartBuf |= 0x0100;
       timOutTxId = 50;
        while(timOutTxId--);
       HAL_UART_Transmit(&huart2,(uint8_t*)&rxUsartBuf,1,HAL_MAX_DELAY);               
     break;
     
      
     
     default:
      ;;
   break;  
     
 }
    
    
      if((nSendByteNDM) && ((rxUsartBuf>=0xd0)&&(rxUsartBuf <= 0xdf))){
          
          tmp_txByte = (0x0100 | *rdParamUart[rxUsartBuf]);
          HAL_UART_Transmit(&huart2, (uint8_t*)&tmp_txByte, 1,HAL_MAX_DELAY);  
          if(nSendByteNDM)nSendByteNDM--;
          (*rdParamUart[rxUsartBuf])=0xff;
           
       }else
         if((nSendByteMIR) && ((rxUsartBuf >= 0x90) && (rxUsartBuf <= 0x9d)))
         {
            tmp_txByte = (0x0100 | *rdParamUart[rxUsartBuf]);
            HAL_UART_Transmit(&huart2, (uint8_t*)&tmp_txByte, 1,HAL_MAX_DELAY);  ;  
            if(nSendByteMIR)nSendByteMIR--;
            (*rdParamUart[rxUsartBuf])=0xff;
         }
      

 
  /* USER CODE END USART2_IRQn 1 */
}



/**
* @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
*/
//41 mcs
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
 
  /* USER CODE END TIM6_DAC_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim6);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  if(!fSinchr)
  {
   
        memcpy(&dataSS[0],&dataSS[1],(SYNCHR_LEHGTH-1)*sizeof(uint16_t));
        dataSS[SYNCHR_LEHGTH-1] = rdDatFromAD7980(); // debugData[indPConRdOutData++]; //rdDatFromAD7980();
        if(findSSinhr(&dataSS[0]))
        {   
          GPIOB->BSRR = GPIO_PIN_3;  
          fSinchr=1;
          indRecWordNdm=SYNCHR_LEHGTH;                       
        }
     
  }else 
  {    
    
      if (indRecWordNdm<nBytesInCadr)
      {
          rxNdmCadr[indRecWordNdm++] = rdDatFromAD7980(); //debugData[indPConRdOutData++] ;//= rdDatFromAD7980(); 
      }
      else
      {
        
        HAL_TIM_Base_Stop_IT(&htim6);
        fSinchr = 3;
        
      }
 
  } 
   

/* USER CODE END TIM6_DAC_IRQn 1 */
}





//uint8_t tstCorData[14]={0x84,0x83,0x04,0x01,0x18,0x01,0x2C,0x01,0x64,0,0xE8,0x03,0x4A,0x01};//<-----
/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim7);
  __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
  /* USER CODE BEGIN TIM7_IRQn 1 */ 
  
  
  if(cntEnWriteToFlash < 40){    
    ++cntEnWriteToFlash;    
  }else{
    enWrDataToFlash = 1;
    cntEnWriteToFlash = 0;
  }
   
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void rxCmndRdFlash(void)
{
   
  uint8_t timOUT;
  
  
  
  for(uint8_t i = 0; i < 8; ++i)
  {
    
    timOUT = 150; isByteRecived = 0;
    while((!isByteRecived)&&(timOUT--)){
      HAL_Delay(1);
    }
    

    if(rxUsartBuf != 0x00CD)
    { 
      rxUsartBuf &= 0xfeff;
      rxUsartBuf |= 0x0100;
      HAL_UART_Transmit(&huart2,(uint8_t*)&rxUsartBuf,1,HAL_MAX_DELAY);
    }
 
  }   
  
  if(((TarirCMND)(dataUsart & 0x000000000000ffff)) == RD_FLASH){
 
      startDataFlash = (uint32_t)(dataUsart >> 40);
      endDataFlash  = (uint32_t)((dataUsart & 0x000000ffffff0000) >> 16);
      HAL_Delay(50);
      readDataFromFlash(); 
  }
  
  dataUsart = 0; 
  
}

void rxCmndSetTime(void)
{
   
  uint32_t timOUT;
 
  
  for(uint8_t i = 0; i < 11; ++i)
  {
    
    timOUT = 150; isByteRecived = 0;
    while((!isByteRecived)&&(timOUT--)){
      HAL_Delay(1);
    }
    
    if(i<6)dataRTC[i] = rxUsartBuf;
    

    if(rxUsartBuf != 0x00ef){    
       rxUsartBuf &= 0xfeff;
       rxUsartBuf |= 0x0100;
       HAL_UART_Transmit(&huart2,(uint8_t*)&rxUsartBuf,1,HAL_MAX_DELAY);}
 
  }
    
  if(((TarirCMND)dataUsart & 0x000000ffffffffff)==ENOWNREQ){
    
     setTimeRTC();
     
  }
  
  dataUsart = 0;  
 
  
}
 

void rxCmndRdTime(void)
{
   
  uint8_t timOUT;
  uint16_t tmp_tx_time; 
  
  GetTimeRTC();
 
  for(uint8_t i = 0; i < 11; ++i)
  {
    
    timOUT = 150; isByteRecived = 0;
    while((!isByteRecived)&&(timOUT--))
    {
      HAL_Delay(1);
    }
    

    if((rxUsartBuf == 0x00C9)&&(i<6)){  
        
      tmp_tx_time = (0x0100 | dataRTC[i]);
      HAL_UART_Transmit(&huart2,(uint8_t*)&tmp_tx_time,1,HAL_MAX_DELAY);
        
    }else if(rxUsartBuf == 0x00ef){
      
      break;
      
    }
 
  }  
  
  dataUsart = 0; 
   
  
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
