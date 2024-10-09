/**
  ******************************************************************************
  * File Name          : SPI.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "i2c.h"
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
 
#define NPOINTS  ((uint8_t)10) 
   
#define np1                   ((uint8_t)(NPOINTS*2)-2)
#define np2                   ((uint8_t) np1+2)
#define np3                   ((uint8_t) NPOINTS-1)

#define chipL         GPIOA->BRR = GPIO_PIN_4     //select flash
#define chipH         GPIOA->BSRR = GPIO_PIN_4    //release flash
   
   

#define NBYTESFLASH     ((uint32_t)0x00ffffff)      //size of flash in bytes
#define ADRPOINTFLASH   ((uint32_t)0)               //address to store a pointer
#define STARTADRFLASH   ((uint32_t)0)               //address to store a pointer


#define WRITEEN        ((uint8_t)0x06)      //write enable 
#define WRITEDIS       ((uint8_t)0x04)     //write disable   
#define FOURBYTEMODE   ((uint8_t)0xB7)     //enter 4 byte mode enter
#define THRRBYTEMODE   ((uint8_t)0xE9)     //Exit 4 byte mode enter

#define READFLASH       ((uint8_t)0x13)     //  READ 4byte mode    
#define PAGEPROG        ((uint8_t)0x12)     //Page programm 4byte mode  
#define SUBSECEREASE    ((uint8_t)0x21)     //SUBSECTOR erease 4byte mode
#define SECEREASE       ((uint8_t)0xDC)     // sector erease 4byte mode
#define THRSUBSECCLR    ((uint8_t)0x20)     //SUBSECTOR erease 3byte mode
#define THRSECEREASE    ((uint8_t)0xD8)     // sector erease 3byte mode
#define THRPAGEPROG     ((uint8_t)0x02)     //Page programm 3byte mode  
#define THRMODERDFLASH  ((uint8_t)0x03)     //READ 3byte mode 


#define RDNONVALCONFREG    ((uint8_t)0xB5)      //write enable    
#define WREXTADRREG        ((uint8_t)0xC5)      //write enable 
#define RDEXTADRREG        ((uint8_t)0xC8)     //write disable

#define  SUBSECSIZE     ((uint32_t)0x00001000)     // subsector size in bytes
#define  SECSIZE        ((uint32_t)0x00010000)     // sector size in bytes

#define  SIZEFORMATFLASH      ((uint32_t)64)         //IN BYTES
#define  INDENDSTRFLASH       ((uint32_t)SIZEFORMATFLASH - 2)         //IN BYTES
#define  CNTSUBSECINFLASH     ((uint32_t)(NBYTESFLASH+1)/SUBSECSIZE) //size of flash in bytes

 
#define  ENDSTRINGFLASH       ((uint16_t)0x0403)  

#define   NDM_WRQUEUE     ((uint8_t)0x01)     
#define   MIR_WRQUEUE     ((uint8_t)0x06)  
#define   MIK_WRQUEUE     ((uint8_t)0x07)  

#define VER_RDFLASH_FORMAT 0x01
#define ID_DEV 0x03


#define PRIZNAK_NDM_SEVENCHANEL 0x00
#define PRIZNAK_NDM_FOURTHCHANEL 0x01
#define PRIZNAK_MIK_ZTK 0x07
#define PRIZNAK_MIR_GEOMASH 0x06


typedef enum{ 
  
  rdFLASH   = 0xCD,    
  clrFLASH  = 0xCB,   
  setTIME   = 0xCA,   
  zamer     = 0x56,  
  rdTIME    = 0xC9,
  rdSOFTVER = 0x59,
  noTASK    = 0x00,
  
} taskTARIR; 

typedef struct{
   uint32_t  currentADR;   
   uint32_t  fullDevolopment;
}tAdrToWrite;

typedef struct{

uint16_t   ZenitUpit;       
uint16_t   GKup;            
uint16_t   GKbot;           
uint16_t   rotationSpeed;   
uint16_t   pressAxial;      
uint16_t   RezKs;            
uint16_t   pressZatrub;     
 
}tDataFromNDM;


typedef struct{
  
uint16_t   zenitIKUpit;       
uint16_t   A1;                 
uint16_t   A2;               
uint16_t   temprMIK;       
uint16_t   A3;                
uint16_t   A4;               
uint16_t   PZatrub;           

}tdatFromMIR;

typedef struct{
  
uint16_t         numPackage;      
uint8_t          dataTime[6];  
tDataFromNDM     dtNDM;
tdatFromMIR      dtMIR;
uint16_t         Gx;       
uint16_t         Gy;                 
uint16_t         Gz;               
uint16_t         levEMCorRec;       
uint16_t         levTRANSFCorRec;                
uint16_t         Otkl;               
uint16_t         zenit; 
uint16_t         tempr;                
uint16_t         idPack;               
uint16_t         verProg; 
uint16_t         regimChanel;               
uint16_t         NarabTek;     
uint16_t         NaraBTotal;   
uint16_t         endString;   

}tdataCorPriemn;
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

void MX_TIM6_Init(void);
void  MX_TIM7_Init(void);
 
extern tAdrToWrite writeAdrFlash;
extern uint16_t totalDevelopment;
extern uint8_t enWrDataToFlash;

void readDataFromFlash();
void clearFlashFull();
void WrToflash(void);
void UARTSendData(const uint8_t *in, uint32_t len,uint32_t deltime);
uint16_t rdDatFromAD7980(); 
void readFSR(void); 
void readSR(void);
void flashRD(uint32_t adr, uint8_t *dOut,const uint32_t nbytes); 
void ereaseFlash(uint32_t adr, uint8_t cmnd);
void wrFlash(uint32_t adr, uint8_t *dIN,const uint32_t nbytes);
void writeCmnd(uint8_t cmnd );
void FlashInit(void);
void switchToSoftSPI(void);
void switchToHardSPI(void);
 //int compareTime(ftime_t realTime,ftime_t lastFlashTime);

 
enum{
  
  SEVEN_CHANEL_MODE = 0x00,
  HALF_CHANEL_MODE = 0x01,
  UNDEF_CHANEL_MODE = 0xFF
};

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
