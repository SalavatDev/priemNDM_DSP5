/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "spi.h"
   
#define NUM_PARAMS_NDM     ((uint16_t)7)
#define NUM_POINTS_INBITS  ((uint16_t)8)
   
#define NUM_POINTS_INBYTE  ((uint16_t)NUM_POINTS_INBITS*8) 
#define SYNCHR_LEHGTH      ((uint16_t)NUM_POINTS_INBYTE*3)
#define NUMBYTE_INSEVEN_CHANELMODE  ((uint8_t)14) 
#define NUM_POINTS_INCADR  ((uint8_t)NUM_POINTS_INBYTE*19) 
#define NUM_POINTS_INHALF  ((uint8_t)NUM_POINTS_INBYTE*13) 

   
#define IWDG_PRESCALER_64   (0x4UL << 0)                         
   
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
typedef enum{ 
  
 
  STCMNDRDFLASH = 0x000000000000CD00 | ID_DEV,
  RDTIME        = 0x0000C9C9C9C9C900 | ID_DEV, 
  CLEAR_FLASH   = 0x0000CBCBCBCBCB00 | ID_DEV,
  SET_TIME      = 0x0000CACACACACA00 | ID_DEV, 
  SET_TIME_ALL  = 0x0000CACACACACA00, 
  ENOWNREQ      = 0x000000EFEFEFEFEF,
  RD_FLASH      = 0x000000000000CDCD,
  RD_SOFTVER    = 0x0000C2C2C2C2C200 | ID_DEV,
  ZAMER         = 0x0000000000000056, 
 
} TarirCMND; 


typedef struct
{
  uint32_t Prescaler;  /*!< Select the prescaler of the IWDG.
                            This parameter can be a value of @ref IWDG_Prescaler */

  uint32_t Reload;     /*!< Specifies the IWDG down-counter reload value.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 0x0FFF */

  uint32_t Window;     /*!< Specifies the window value to be compared to the down-counter.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 0x0FFF */

} IWDG_InitTypeDef;


typedef struct
{
  IWDG_TypeDef                 *Instance;  /*!< Register base address    */

  IWDG_InitTypeDef             Init;       /*!< IWDG required parameters */
} IWDG_HandleTypeDef;

 
#define  BAUDRATE        ((uint32_t)172800) //range 167000-172800 kb/s
#define  IDDEVICE         (uint8_t)3
#define  IDTOTAL          (uint8_t)0



typedef enum
{
   INVERSE_PHASE = 0x01,
   NORMAL_PHASE = 0x00,
   UNDEF_PHASE = 0x02,
   
}typePhase_TD;

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

extern IWDG_HandleTypeDef hiwdg; 

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void send_byteUART3(uint8_t data);
void allNDMDataRecive(void);
void MX_IWDG_Init(void);
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
