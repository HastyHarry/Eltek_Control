/**
  ******************************************************************************
  * @file    : DPC_Miscellaneous.h
  * @brief   : Miscellaneous fuction management
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
#ifndef __DPC_MISCELLANEOUS_H
#define __DPC_MISCELLANEOUS_H


/* Includes ------------------------------------------------------------------*/
#include "DPC_Lib_Conf.h"
#include "DPC_Datacollector.h"
#include "DPC_adc_converter.h"


/* Exported types ------------------------------------------------------------*/


/**
 *@brief
 */
typedef enum
{
  NO_LOAD = 0,                          /*!< */
  LOW_LOAD,                             /*!< */
  ON_LOAD,                              /*!< */
  OVERVOLTAGE_LOAD,                     /*!< */
  OVERCURRENT_LOAD,                     /*!< */
  OVERVOLTAGE_CAP,                      /*!< */
} DPC_Load_Status_TypeDef;


/**
 *@brief
 */
typedef enum
{
  WAIT_SOURCE = 0,                      /*!< */
  NO_SOURCE,                            /*!< */
  UV_SOURCE,                            /*!< */
  UVLO_SOURCE,                          /*!< */  
  OK_SOURCE,                            /*!< */
  OVERVOLTAGE_SOURCE,                   /*!< */
  OVERCURRENT_SOURCE,                   /*!< */
  FAULT,
}DPC_Source_Status_TypeDef;

/**
 *@brief
 */
typedef enum
{
  NO_Plug_ACSource = 0,                 /*!< */
  OK_Plug_ACSource,                     /*!< */
}DPC_Status_Plug_ACSource_TypeDef;


/**
 *@brief Relay: Variables
 */
typedef enum
{
  RELAY_GRID_A = 0x01,                  /*!< Grid Relay Phase A*/
  RELAY_GRID_B = 0x02,                  /*!< Grid Relay Phase B*/
  RELAY_GRID_C = 0x04,                  /*!< Grid Relay Phase C*/
  RELAY_GRID_ABC = 0x7,                 /*!< All Grid Relay Phase ABC*/
  RELAY_SER_1 = 0x08,                   /*!< */
  RELAY_SER_2 = 0x10,                   /*!< */
  RELAY_SER_3 = 0x20,                   /*!< */
  RELAY_SER_4 = 0x40,                   /*!< */
  RELAY_FAN = 0x80,                     /*!< */
  RELAY_ALL = 0xFF,                     /*!< All Relays*/
} DPC_Relay_TypeDef;


/**
 *@brief LEDs: Variables
 */
typedef enum
{
  BLED_Idle = 0x00,                    /*!< */
  BLED_StartUp_inrush = 0x01,          /*!< */
  BLED_Fault = 0x02,                   /*!< */
  BLED_Error = 0x3,                    /*!< */
  BLED_Run = 0x04,                     /*!< */
  BLED_StartUp_burst = 0x5,            /*!< */
  BLED_Stop = 0x6,                     /*!< */
  BLED_Debug = 0x7,                    /*!< */  
  BLED_Wait = 0x8,                     /*!< */    
} DPC_BLED_TypeDef;

  
/**
 *@brief DC Load: Variables & Limit
 */
typedef struct {
uint16_t V_cap_Limit;                   /*!< */
uint16_t I_No_load_Threshold;           /*!< */
uint16_t I_No_load_Max_Threshold;       /*!< */
uint16_t I_No_load_Min_Threshold;       /*!< */
uint16_t I_Low_load_Threshold;          /*!< */
uint16_t I_Low_load_Max_Threshold;      /*!< */
uint16_t I_Low_load_Min_Threshold;      /*!< */
uint16_t I_Over_load_Threshold;         /*!< */
uint16_t V_dc_Limit;                    /*!< */
}DPC_Load_Limit_TypeDef;

/** 
 *@brief AC Source: Variables & Limit
 */
typedef struct {
uint16_t V_ac_pos_Limit;                /*!< */
uint16_t V_ac_neg_Limit;                /*!< */
uint16_t V_ac_pos_UVLO_Limit;           /*!< */
uint16_t V_ac_neg_UVLO_Limit;           /*!< */
uint16_t V_ac_pos_UV_Limit;             /*!< */
uint16_t V_ac_neg_UV_Limit;             /*!< */
uint16_t V_ac_pos_Low_Limit;            /*!< */
uint16_t V_ac_neg_Low_Limit;            /*!< */
uint16_t I_ac_pos_Limit;                /*!< */
uint16_t I_ac_neg_Limit;                /*!< */
}DPC_Source_Limit_TypeDef;



/** 
 *@brief AC Source Variables & Limit
 */
typedef struct {
uint16_t V_ac_pk_pos_local;                     /*!< */
uint16_t V_ac_pk_neg_local;                     /*!< */
uint16_t I_ac_pk_pos_local;                     /*!< */
uint16_t I_ac_pk_neg_local;                     /*!< */
DPC_Source_Status_TypeDef Status_Source;        /*!< */
}DPC_Source_TypeDef;


/** 
 *@brief DC Load 
 */
typedef struct {
DPC_Load_Status_TypeDef DPC_Load_Status;/*!< */
}DPC_Load_TypeDef;


/* Exported constants --------------------------------------------------------*/

#define  RELAY_OFF GPIO_PIN_RESET
#define  RELAY_ON  GPIO_PIN_SET
#define  DPC_Relay_State_TypeDef GPIO_PinState

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//DPC_Load_Status_TypeDef DPC_MISC_Check_DCLoad(DPC_Load_TypeDef *DPC_Load_loc,DPC_Load_Limit_TypeDef DC_Load_Limit_sub);
DPC_Load_Status_TypeDef DPC_MISC_Check_DCLoad(DPC_Load_TypeDef *DPC_Load_loc,DPC_Load_Limit_TypeDef DC_Load_Limit_sub, CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub);
DPC_Source_Status_TypeDef DPC_MISC_CHECK_AC_SOURCE(DPC_Source_TypeDef *AC_Source_sub,DPC_Source_Limit_TypeDef AC_Source_Limit_sub,float Theta);
void DPC_MISC_ACSource_Init(DPC_Source_Limit_TypeDef *AC_Source_Limit_sub,uint16_t V_ac_Limit_VOLT,uint16_t V_ac_UV_Limit_VOLT,uint16_t V_ac_UVLO_Limit_VOLT,uint16_t V_ac_Low_Limit_VOLT,uint16_t I_ac_Limit_AMP,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc);
DPC_Status_Plug_ACSource_TypeDef DPC_MISC_AC_SOURCE_Plugged(DPC_Source_Limit_TypeDef AC_Source_Limit_sub);
void DPC_MISC_DCLoad_Init(DPC_Load_Limit_TypeDef *DC_Load_Limit_sub,uint16_t V_dc_Limit_VOLT,uint16_t V_cap_Limit_VOLT,float I_dc_NO_LOAD_Limit_AMP,float I_dc_LOW_LOAD_Limit_AMP,float I_dc_OVER_LOAD_Limit_AMP,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc);
void DPC_MISC_APPL_Timer_Init(TIM_HandleTypeDef AppTIM, uint32_t  APPL_Freq_Desidered);
void DPC_MISC_Appl_Timer_Start(void);
void DPC_MISC_BLED_Set(TIM_HandleTypeDef *htim_bled,uint32_t TIM_CHANNEL_BLED,DPC_BLED_TypeDef State_BLED);
void DPC_MISC_RELAY_Cntl(DPC_Relay_TypeDef Relay_n, DPC_Relay_State_TypeDef bState);
void DPC_DAC_Init(DAC_Channel_STRUCT *DAC_CH_Sub, uint8_t DAC_CH1_INIT_loc, uint8_t DAC_CH2_INIT_loc, uint8_t DAC_CH3_INIT_loc, uint16_t DAC_G_CH1_Init_loc,uint16_t DAC_G_CH2_Init_loc , uint16_t DAC_G_CH3_Init_loc, uint16_t DAC_B_CH1_Init_loc,uint16_t DAC_B_CH2_Init_loc , uint16_t DAC_B_CH3_Init_loc);
#endif /* __DPC_MISCELLANEOUS_H */
