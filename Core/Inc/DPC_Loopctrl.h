/**
  ******************************************************************************
  * @file    Loop_Ctrl.h
  * @brief   This file contains the headers of the Loop_Ctrl Module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPC_LOOPCTRL_H
#define __DPC_LOOPCTRL_H

/* Includes ------------------------------------------------------------------*/

#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif

#include "DPC_Datacollector.h"
#include "DPC_PWMConverter.h"
#include "DPC_adc_converter.h"



/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Relays Structure definition
  */
  typedef struct {
    FlagStatus RELAY_INRSH_State;
    FlagStatus RELAY_GRID_A_State;
    FlagStatus RELAY_GRID_B_State;
    FlagStatus RELAY_GRID_C_State;
    FlagStatus FAN_State;    
  }
  Relay_Typedef;



/**
  * @brief  Current Decoupling Control Structure definition
  */
typedef struct{
  float omegagrid;                                              /*!<Omega grid value expressed in Hz - Related to decoupled terms*/
  float Inductor;                                               /*!<Inductor value expressed in H - Related to decoupled terms*/
  float Id_ref;                                                 /*!<d-axis current referance*/
  float Iq_ref;                                                 /*!<q-axis current referance*/
  float Id_feed;                                                /*!<d-axis current feedback*/
  float Iq_feed;                                                /*!<q-axis current feedback*/
  float kp_pll;                                                 /*!< Proportional Gain*/
  float ki_pll;                                                 /*!< Integral Gain*/
  float Vd_Curr_Ctrl;                                           /*!< d-axis - Output of the decoupling compensation*/
  float Vq_Curr_Ctrl;                                           /*!< q-axis - Output of the decoupling compensation*/
  float Vd_feed;                                                /*!< d-axis - AC main feedforward term*/
  float Vq_feed;                                                /*!< q-axis - AC main feedforward term*/
  float Vd_Decoupling;                                          /*!< d-axis - Decoupling terms (iq*omega*L)*/
  float Vq_Decoupling;                                          /*!< q-axis - Decoupling terms (iq*omega*L)*/
  float Vdc_feed;                                               /*!< Vdc feedforward term*/
  float Vd_ctrl_FF;                                             /*!< d-axis - CDC voltage control output*/ 
  float Vq_ctrl_FF;                                             /*!< q-axis - CDC voltage control output*/
  FlagStatus FF_Enable;                                         /*!< AC voltage FeedForward Enabling*/
  FlagStatus Decoupling_Enable;                                 /*!< AC currents Decoupling Enabling*/
  FlagStatus VDC_FF_Enable;                                     /*!< DC voltage FeedForward Enabling*/
  PI_STRUCT_t pPI_ID_CURR_CTRL;                                 /*!< d-axis - PI regulator STRUCT*/
  PI_STRUCT_t pPI_IQ_CURR_CTRL;                                 /*!< q-axis - PI regulator STRUCT*/
}CDC_Struct;


/**
  * @brief  DC Voltage Control Structure definition
  */
typedef struct{
  float Vdc_ref;
  float Vdc_feed;
  float Id_ctrl;
}VOLTAGECTRL_Struct;


/**
  * @brief  Inrush Current Control Structure definition
  */
typedef struct {
uint16_t Vout_load;                                             /*!< Output DC voltage expressed in Bits*/     
uint16_t Vref_hist;                                             /*!< Histeresis ouput DC voltage Thrueshold expressed in Bits*/
uint16_t delta_Vref_hist;                                       /*!< Delta histeresis ouput DC voltage Thrueshold expressed in Bits*/
uint16_t Vout_max;                                              /*!< Histeresis higher ouput DC voltage Thrueshold expressed in Bits */
uint16_t Vout_min;                                              /*!< Histeresis lower ouput DC voltage Thrueshold expressed in Bits*/
FlagStatus InrushEnable;                                        /*!< Inrush Enabled flag*/
uint16_t Iout_load_threshold;                                   /*!< Ouput DC current Thrueshold expressed in Bits*/
uint16_t I_load_Inrush;                                         /*!< */
INRUSH_StatusTypeDef INRUSH_Status;
}INRUSH_STRUCT;


/**
  * @brief  Type of control activated OPEN - INNER - OUTER
  */
typedef enum
{
  OPEN_LOOP = 0,
  CURRENT_LOOP,
  VOLTAGE_LOOP,
 } PFC_CTRL_State_TypeDef;


/**
  * @brief  
  */
typedef struct{
  FlagStatus CDC_Reset;
  FlagStatus VdcCTRL_Reset;
  PFC_CTRL_State_TypeDef PFC_CTRL_State;
  TRANSFORM_QDO_t V_DQO_CTRL_MAN;
  uint16_t PFC_VDC_Ref;
  uint16_t PFC_VDC_Ref_BITs;
  CDC_Struct CDC;
  VOLTAGECTRL_Struct VOLTAGECTRL;
}PFC_CTRL_t;
    

void Current_Decoupling_Control(CDC_Struct *pCDC_sub,PI_STRUCT_t *pPI_ID_CURR_CTRL_sub, PI_STRUCT_t *pPI_IQ_CURR_CTRL_sub , float *pVd_ctrl_FF_sub, float *pVq_ctrl_FF_sub);
void FeedForward_Control(CDC_Struct *pCDC_sub,float *pVd_ctrl_FF_sub, float *pVq_ctrl_FF_sub);
void Voltage_Control(VOLTAGECTRL_Struct *pVOLTAGECTRL_sub,PI_STRUCT_t *pPI_VDC_CTRL_sub, float *pId_ctrl_sub);
void DPC_LPCNTRL_PFC_Mode_Reset(PI_STRUCT_t *PI_VDC_CTRL, CDC_Struct *CDC);
void DPC_LPCNTRL_PFC_Mode(PFC_CTRL_t *pPFC_CTRL_loc, PI_STRUCT_t *pPI_VDC_CTRL, VOLTAGECTRL_Struct *VOLTAGECTRL, CDC_Struct *CDC,TRANSFORM_QDO_t *V_DQO_CTRL, TRANSFORM_QDO_t *Current_qdo,TRANSFORM_QDO_t *Voltage_qdo,VoltageDC_ADC_NORM_Struct *VOLTAGE_ADC_AC_IN_PHY);
void DPC_LPCNTRL_BURST_Init(BURST_STRUCT *BURST_t_local,FlagStatus Burst_Enable_loc,uint16_t Vref_hist_loc,uint16_t delta_Vref_hist,float I_dc_NO_LOAD_Limit_AMP_loc,float I_dc_LOW_LOAD_Limit_AMP_loc,float duty_local,float duty_no_load_local,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc);
BURST_StatusTypeDef DPC_LPCNTRL_Burst_Check(uint32_t* p_Data_Sub,uint32_t* iDC_Data_Sub,BURST_STRUCT *BURST_CTRL_f);
void DPC_LPCNTRL_CDC_Init(CDC_Struct *CDC,float omegagrid_loc,float Inductor_loc,FlagStatus FF_Enable_SET,FlagStatus Decoupling_Enable_SET,FlagStatus VDC_FF_Enable_SET);
void DPC_CTRL_RELAY_ChangeState(Relay_Typedef *Relay_local);
void DPC_Relay_Init(Relay_Typedef *Relay_local);
void DPC_LPCNTRL_Burst_Mode(uint32_t* p_Data_Sub,BURST_STRUCT *BURST_CTRL_f,uint32_t* iDC_Data_Sub,DPC_PWM_TypeDef *tDPC_PWM_loc);
INRUSH_StatusTypeDef DPC_LPCNTRL_Inrush_Check(uint32_t* p_Data_Sub,uint32_t* iDC_Data_Sub,INRUSH_STRUCT *INRUSH_CTRL_f);
void DPC_LPCNTRL_Inrush_Init(INRUSH_STRUCT *INRUSH_CTRL_f,uint16_t Vref_hist_VOLT_loc,uint16_t delta_Vref_hist_VOLT_loc,float I_dc_NO_LOAD_Limit_AMP_loc,FlagStatus InrushEnable_loc,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc);
void DPC_LPCNTRL_PFC_Init(PFC_CTRL_t *pPFC_CTRL,PFC_CTRL_State_TypeDef PFC_CTRL_State,uint16_t PFC_VDC_Ref_loc,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc);
#endif /*__DPC_LOOPCTR.H */