/**
  ******************************************************************************
  * @file    DPC_LUT.h
  * @brief   This file contains the headers of the DPC_LUT Module.
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
#ifndef __DPC_LUT_H
#define __DPC_LUT_H


/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

/* Exported types ------------------------------------------------------------*/


typedef struct {

//uint16_t ramp; //uint16_t ramp=0;
float ramp; //uint16_t ramp=0;
uint16_t rampA;
uint16_t rampB;
uint16_t rampC;
uint16_t ramp_offset;
uint16_t uh_fgrid_Hz;                      /*!< Grid Frequency (uint16_t) expressed in [Hz]*/
float ma;
//uint16_t step;   //uint16_t step=0;
float step;   //uint16_t step=0;
float discretization_coefficient;
}treephaseSTRUCT;


#define LUTSAMPLE 200
#define STDES_PFCBIDIR_REV2
typedef struct {
#if defined(STDES_PFCBIDIR) || defined(STDES_VIENNARECT) || defined(STDES_PFCBIDIR_REV2)
uint16_t LUTramp[LUTSAMPLE];
uint16_t LUTsinePOS[LUTSAMPLE];
uint16_t LUTsineNEG[LUTSAMPLE];
float LUTsinePOSfloat[LUTSAMPLE];
float LUTsineNEGfloat[LUTSAMPLE];
float LUTramp_2pi[LUTSAMPLE];
float LUTsinefloat[LUTSAMPLE];
#elif STDES_PFCBIDIR_2L  
uint16_t LUTsine[LUTSAMPLE];
float LUTsine01[LUTSAMPLE];
#else    
SELECT DEFINE
#endif
}lutSTRUCT;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DPC_LUT_Gen(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *LUT_3PH_sub,uint32_t  PWM_PERIOD_COUNTER);
void DPC_LUT_Gen_opt(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *LUT_3PH_sub,uint32_t  PWM_PERIOD_COUNTER);
void Gen_LUT_FASTMATH(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *LUT_3PH_sub,uint32_t  PWM_PERIOD_COUNTER);
void DPC_LUT_Init(treephaseSTRUCT* LUT_STRUCT,uint32_t  PWM_PERIOD_COUNTER,uint32_t RateRead,uint32_t SAMPLE,uint16_t uh_fgrid_Hz_local,float ma_local);

 
#endif //__DPC_LUT_H
