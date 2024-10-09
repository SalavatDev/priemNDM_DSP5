/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "usart.h"
/*
char    debString[64]=//<-------------------------------
{
  ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
  ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
  ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
  ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '
}; 
 
extern UART_HandleTypeDef huart3; //<-------------------------------
extern  uint16_t  numWrAdrFlash; //<-------------------------------
extern uint16_t  cntPack; //<-------------------------------
extern int cmprTim; //<-------------------------------
extern ftime_t  utcToTime;//<-------------------------------
*/
uint8_t dataRTC[6]={0,0,0,0,0,0};
 
uint8_t dataInit_RTC=0x04;
 
const uint8_t DayMonthInVisYear[12]={31,29,31,30,31,30,31,31,30,31,30,31};
const uint8_t DayMonthInYear[12]={31,28,31,30,31,30,31,31,30,31,30,31};

const uint16_t day_offset[12] = {0, 31, 61, 92, 122, 153, 184, 214, 245, 275, 306, 337};

const uint8_t   adr_RTC=0;  
extern TIM_HandleTypeDef htim6;
 
extern   __IO uint32_t  currAdrFlash ;
 
ftime_t typeDataTime;

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40707EB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void initRTC(void){

   uint8_t dataInit_RTC[2]={0x0E,0x04}; //настройка часов отключаем пин INT
   HAL_I2C_Master_Transmit(&hi2c1, 0xd0, (uint8_t *)&dataInit_RTC[0] , 2, 1); 
  
}



//----функция перевода времени из "обычного вида" в формат UTC  
 
uint32_t rtc_code()
{  
  
   uint32_t result=0, cntYearVis, cntYear, cntDay ;
   uint8_t isVisYear;
 
 if(!((2000+typeDataTime.year)%4))
   {
     isVisYear = 1;
   }else
   {
     isVisYear = 0;
   }
   
  cntYear = (30 + typeDataTime.year); 
  cntYearVis = cntYear/4;
  result =  cntYearVis + cntYear * 365;
  cntDay=0;
  for(uint8_t i=0;i<typeDataTime.month-1;i++)
  {
    if(isVisYear)
      cntDay += DayMonthInVisYear[i];
    else 
      cntDay += DayMonthInYear[i];
  }
    
   result = (result + cntDay + typeDataTime.day - 1)*86400;
  
   if(!((2000+typeDataTime.year-1)%4) )
   {
     result += 86400;
   }    

  result += (typeDataTime.hour * 3600) + (typeDataTime.minute * 60) + typeDataTime.second;
  
  return result;
  
}

 

/*
void logInfoTx( uint8_t wrOrPr)//<-------------------------------
{
  uint8_t i;
  if(wrOrPr==1)
  {
    sprintf(debString,"prAt %d:%d:%d  %d %d %d %d %d\n",  typeDataTime.hour, typeDataTime.minute, typeDataTime.second, typeDataTime.day, typeDataTime.month, currAdrFlash,numWrAdrFlash,cntPack);
  }
  else if(wrOrPr==2)
  {
    sprintf(debString,"WakeUp %d:%d:%d %d.%d.%d  %d:%d:%d %d.%d.%d\n",  typeDataTime.hour, typeDataTime.minute, typeDataTime.second, typeDataTime.day, typeDataTime.month, typeDataTime.year,utcToTime.hour,utcToTime.minute,utcToTime.second,utcToTime.day,utcToTime.month,utcToTime.year);
  }
  else
  {
    sprintf(debString, "wr %d:%d:%d  %d %d %d %d %d\n",  typeDataTime.hour, typeDataTime.minute, typeDataTime.second, typeDataTime.day, typeDataTime.month, currAdrFlash,numWrAdrFlash,cntPack);
  }
   
  i=0;
  while(debString[i]!=0)
  {
    send_byteUART3((uint8_t)debString[i++]);
  }
   // HAL_UART_Transmit(&huart3,(uint8_t*)&debString[0],23,15);  
    //memset(debString,' ',23);
    
}
*/

void setTimeRTC(){
  
  uint8_t tmpTime[8]={0,0,0,0,0,0,0,0};
  memcpy(&tmpTime[1],&dataRTC[0],3);
  tmpTime[4]=0;
  memcpy(&tmpTime[5],&dataRTC[3],3);
  HAL_IWDG_Refresh(&hiwdg);

  HAL_I2C_Master_Transmit(&hi2c1, DEVADRRTC, (uint8_t *)&tmpTime[0] , 8, 50);
  memset(&dataRTC[0],0,6);
  
}


extern HAL_StatusTypeDef status_I2C;

void GetTimeRTC(){
  
   uint8_t rtcTime[7]={0,0,0,0,0,0,0 };
  
   HAL_IWDG_Refresh(&hiwdg);
     
   HAL_I2C_Master_Transmit(&hi2c1, DEVADRRTC, (uint8_t *)&adr_RTC , 1, 50);
   status_I2C = HAL_I2C_Master_Receive(&hi2c1, DEVADRRTC, (uint8_t *)&rtcTime ,7, 50);
    
   HAL_Delay(2);   
   
   dataRTC[0] = rtcTime[0];
   dataRTC[1] = rtcTime[1];
   dataRTC[2] = rtcTime[2];
   dataRTC[3] = rtcTime[4];
   dataRTC[4] = rtcTime[5];
   dataRTC[5] = rtcTime[6];  
   
   typeDataTime.second = dataRTC[0]/16*10+ dataRTC[0]%16;
   typeDataTime.minute = dataRTC[1]/16*10+ dataRTC[1]%16; 
   typeDataTime.hour = dataRTC[2]/16*10+ dataRTC[2]%16;
   typeDataTime.day = dataRTC[3]/16*10+ dataRTC[3]%16;
   typeDataTime.month = dataRTC[4]/16*10+ dataRTC[4]%16;
   typeDataTime.year = dataRTC[5]/16*10+ dataRTC[5]%16;
     
 
} 


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
