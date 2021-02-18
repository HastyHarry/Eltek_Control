/**
  ******************************************************************************
  * @file    DPC_Datacollector.h
  * @brief   This file contains the headers of the Database.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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

#ifndef __DPC_DATACOLLECTOR_H
#define __DPC_DATACOLLECTOR_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

#include "main.h"
#include "DPC_LUT.h"

/* Exported types ------------------------------------------------------------*/


/*!FSM*/
typedef enum
{
  FSM_Idle = 0,
  FSM_StartUp_inrush,
  FSM_StartUp_burst,
  FSM_Run,
  FSM_Fault,
  FSM_Error,
  FSM_Debug,
  FSM_Stop,    
} PC_State_TypeDef;


typedef enum
{
  Run_Idle = 0,
  Run_Burst_Mode,
  Run_PFC_Mode
} Run_State_TypeDef;

/*!DAC Channel Struct Declaration*/
typedef struct{
uint8_t CH1;                            /*!< Data connector for DAC Channel 1*/  
uint8_t CH2;                            /*!< Data connector for DAC Channel 2*/
uint8_t CH3;                            /*!< Data connector for DAC Channel 3*/
uint16_t Gain_CH1;                      /*!< Gain Data for DAC Channel 1*/
uint16_t Gain_CH2;                      /*!< Gain Data for DAC Channel 2*/
uint16_t Gain_CH3;                      /*!< Gain Data for DAC Channel 3*/
uint16_t Bias_CH1;                      /*!< Bias Data for DAC Channel 1*/
uint16_t Bias_CH2;                      /*!< Bias Data for DAC Channel 2*/
uint16_t Bias_CH3;                      /*!< Bias Data for DAC Channel 3*/
}DAC_Channel_STRUCT;            



//AC Side
typedef struct{
  uint32_t phA;
  uint32_t phB;
  uint32_t phC;
}THREE_PH_AC_Struct_t;

#define VoltageAC_ADC_Struct THREE_PH_AC_Struct_t
#define CurrentAC_ADC_Struct THREE_PH_AC_Struct_t


typedef struct{
  float phA;
  float phB;
  float phC;
}THREE_PH_AC_NORM_Struct_t;

#define VoltageAC_ADC_NORM_Struct THREE_PH_AC_NORM_Struct_t
#define CurrentAC_ADC_NORM_Struct THREE_PH_AC_NORM_Struct_t

//DC Side
typedef struct{
  float Vdc_pos;
  float Vdc_neg;
  float Vdc_tot;
}VoltageDC_ADC_NORM_Struct;


typedef struct{  
  uint32_t Vdc_pos;
  uint32_t Vdc_neg;
}VoltageDC_ADC_Struct;


typedef struct{  
  uint32_t IDC_adc;
}CurrentDC_ADC_Struct;


typedef struct{
  float IDC_adc;
}CurrentDC_ADC_NORM_Struct_t;


//Reference Frame 
typedef struct{  
  float axA;
  float axB;
  float axC;
}TRANSFORM_ABC_t;


typedef struct{
  float axd;
  float axq;
  float axo;
}TRANSFORM_QDO_t;


typedef struct{
  uint32_t phA_adc;
  uint32_t phB_adc;
  uint32_t phC_adc;
}AC_ADC_Struct;


typedef union{
  AC_ADC_Struct VAC;
  AC_ADC_Struct IAC;
  VoltageDC_ADC_Struct VDC;
} RAW_ADC_UNION;


/* Exported constants --------------------------------------------------------*/
#define DOUBLE_PI 6.28318530718f //Represent 2*Pi in floating point
#define HALF_PI 1.57079632679f //Represent Pi/2 in floating point
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

VoltageAC_ADC_Struct* DATA_Vabc_Read(void);
void DATA_Write_Theta_PLL(float Theta);    
float DATA_Read_Theta_PLL(void);
void DATA_CURR_Write_ClarkePark(TRANSFORM_QDO_t Results_ClarkePark);
void DATA_VOLT_Write_ClarkePark(TRANSFORM_QDO_t Results_ClarkePark);                           
TRANSFORM_QDO_t DATA_CURR_Read_ClarkePark(void);
TRANSFORM_QDO_t DATA_VOLT_Read_ClarkePark(void);
void InitDMA_ADC_CONV(void);
RAW_ADC_UNION* DATA_Read_ADC_Raw(void);
//#ifdef ADC_GRID


//#ifdef STDES_VIENNARECT
//void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data,uint32_t* p_ADC2_Data);
//#elif STDES_PFCBIDIR
//void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data,uint32_t* p_ADC2_Data);
//#elif STDES_PFCBIDIR_REV2
void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data,uint32_t* p_ADC2_Data); 
void DATA_Acquisition_from_LUT(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *RAMP_DATA);
//#elif NUCLEO_3TTC
//void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data);
//#else
//  SELECT DEFINE
//#endif
    
//#endif

VoltageAC_ADC_Struct* Read_GRID(void);
CurrentAC_ADC_Struct* Read_Curr_GRID(void);
CurrentDC_ADC_Struct* Read_Curr_DC(void);
VoltageDC_ADC_Struct* Read_Volt_DC(void);


#endif    /*__DPC_DATACOLLECTOR_H*/


/************************ (C) COPYRIGHT STMicroelectronics 2020 END OF FILE****/
