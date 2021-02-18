/**
  ******************************************************************************
  * @file    DPC_PWMConverter.h
  * @brief   This file contains the headers of the DPC_PWMConverter module.
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
#ifndef __DPC_PWMCONVERTER_H
#define __DPC_PWMCONVERTER_H


/* Includes ------------------------------------------------------------------*/
#include "DPC_Lib_Conf.h"
#include "DPC_Datacollector.h"
#include "DPC_LUT.h"

/* Exported types ------------------------------------------------------------*/
//

//BURST
typedef enum
{
  BURST_Start    = 0x00U,
  BURST_Complete = 0x01U,
  BURST_Error    = 0x02U,
  BURST_Progress = 0x03U,
  BURST_TIMEOUT  = 0x04U,
  BURST_Disable  = 0x05U,
  BURST_Run      = 0x06U  
} BURST_StatusTypeDef;

typedef struct {
uint16_t Vout_load;
uint16_t Vref_hist;
uint16_t delta_Vref_hist;
uint16_t Vout_max;                                              /*!< Histeresis higher ouput DC voltage Thrueshold expressed in Bits */
uint16_t Vout_min;                                              /*!< Histeresis lower ouput DC voltage Thrueshold expressed in Bits*/
FlagStatus BURST_PACKAGE;
FlagStatus BURST_IN_RANGE;
float Duty_noload;
float Duty_lowload;
float Duty_Limit;
FlagStatus Burst_Enable;
uint16_t Iout_no_load_threshold;
uint16_t Iout_low_load_threshold;
BURST_StatusTypeDef BURST_Status;
uint16_t uI_load_Burst;
float Burst_Duty;
}BURST_STRUCT;


typedef enum
{
  INRUSH_Start    = 0x00U,
  INRUSH_Complete = 0x01U,
  INRUSH_Error    = 0x02U,
  INRUSH_Progress = 0x03U,
  INRUSH_TIMEOUT  = 0x04U,
  INRUSH_Disable = 0x05U
} INRUSH_StatusTypeDef;

typedef enum
{
  PWM_Safe        = 0x00U,
  PWM_Armed       = 0x01U,
  PWM_Error       = 0x02U,
  PWM_Fault       = 0x02U,  
} DPC_PWM_StatusTypeDef;

typedef struct
{
	uint32_t phA;
	uint32_t phB;
	uint32_t phC;
}DMA_PWMDUTY_STRUCT;


typedef struct {
DPC_PWM_StatusTypeDef DPC_PWM_Status;
#if defined(STDES_PFCBIDIR)
float VApos;
float VAneg;
float VBpos;
float VBneg;
float VCpos;
float VCneg;
#elif defined(STDES_PFCBIDIR_REV2)
float VApos;
float VAneg;
float VBpos;
float VBneg;
float VCpos;
float VCneg;
#endif
uint32_t dutyMaxLim;
uint32_t dutyMinLim;
uint32_t PWM_Period;
uint32_t BURST_PWM_Period;
}DPC_PWM_TypeDef;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DPC_PWM_InitDUTY(uint16_t  PULSE1_VALUE,uint16_t  PULSE2_VALUE,uint16_t  PULSE3_VALUE);
void DPC_PWM_ADVTIM_PWMStart(void);
void DPC_PWM_ADVTIM_PWMPWMStop(void);
void DPC_PWM_SET_Dead_Time(uint16_t uwDeadTimeSub);
void DPC_PWM_Send_Duty_SPWM(DPC_PWM_TypeDef *tDPC_PWM_loc,float VA,float VB,float VC, DMA_PWMDUTY_STRUCT* DMA_SRC);
void DPC_PWM_Send_Burst_PWM(DPC_PWM_TypeDef *tDPC_PWM_loc,float BURST_A,float BURST_B,float BURST_C);

void Send_Duty_2LC_SPWM(float VA,float VB,float VC,uint32_t  PWM_PERIOD_COUNTER_INT);
void RefreshDuty(uint16_t LUTsinePOS[],uint16_t LUTsineNEG[],treephaseSTRUCT *LUT_3PH,uint32_t  PWM_PERIOD_COUNTER);
void Send_Duty_2LC_SPWM_LUT(uint16_t VA,uint16_t VB,uint16_t VC);
void Send_Duty_2LC_SPWM_OPT(float VA,float VB,float VC,uint32_t  HALF_PWM_PERIOD_COUNTER_INT);
void Send_Duty_2LC_SPWM_OPT(float VA,float VB,float VC,uint32_t  PWM_PERIOD_COUNTER_INT);
void Gen_LUT_FASTMATH(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *LUT_3PH_sub,uint32_t  PWM_PERIOD_COUNTER);
//void Send_Duty_VIENNA_SPWM_OPT(float VA,float VB,float VC,uint32_t  PWM_PERIOD_COUNTER_INT);
void Send_Duty_3LTTC_SPWM_OPT(float VA,float VB,float VC,uint32_t  PWM_PERIOD_COUNTER_INT);
void VIENNA_MOD_VCTRL(TRANSFORM_ABC_t *VIENNA_MOD_ABC_CTRL);
void DPC_PWM_HRTIM_Stop(void);
void DPC_PWM_HRTIM_Start(void);
void DPC_PWM_HRTIM_Init(void);
void DPC_PWM_HRTIM_Set(uint32_t Period,float Duty);
void DPC_PWM_HRTIM_OutEnable(void);
void DPC_PWM_HRTIM_OutDisable(void);
void DPC_PWM_ADVTIM_OutDisable(void);
void DPC_PWM_ADVTIM_OutEnable(void);
void DPC_PWM_Set_HRTIM(float VA,float VB,float VC);
void DPC_PWM_OutDisable(void);
void DPC_PWM_OutEnable(DPC_PWM_TypeDef *tDPC_PWM_loc);
void DPC_PWM_Start(void);
//void DPC_PWM_Init(uint32_t  BURST_PWM_Freq_Desidered,uint32_t  PWM_Freq_Desidered,DPC_PWM_StatusTypeDef DPC_PWM_SET, DPC_PWM_TypeDef *tDPC_PWM_loc);
void DPC_PWM_Init(uint32_t  BURST_PWM_Freq_Desidered,uint32_t  PWM_Freq_Desidered,DPC_PWM_StatusTypeDef DPC_PWM_SET, DPC_PWM_TypeDef *tDPC_PWM_loc, DMA_PWMDUTY_STRUCT *DUTY_SRC);
void DPC_ConfigDeadTime(TIM_HandleTypeDef *htim_HS_AS,TIM_HandleTypeDef *htim_LS_DS,uint32_t Deadtime);
uint8_t DPC_Calc_DTG(float DT_TimeVal);
void DPC_Calc_DTG_Range(float DT_TimeVal, float t_tim_ket_ck);
#endif /* __DPC_PWMCONVERTER_H */
