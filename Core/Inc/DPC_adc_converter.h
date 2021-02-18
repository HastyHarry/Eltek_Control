/**
  ******************************************************************************
  * @file    DPC_adc_converter.h
  * @brief   This file contains the headers of the transform module.
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
#ifndef __ADC_CONVERTER_H
#define __ADC_CONVERTER_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


typedef struct {
float G_Vac;
float invG_Vac;
float B_Vac;
float G_Iac;
float invG_Iac;
float B_Iac;
float G_Vdc;
float invG_Vdc;
float B_Vdc;
float G_Idc;
float invG_Idc;
float B_Idc;
FlagStatus DPC_ADC_Conf_Complete;
}DPC_ADC_Conf_TypeDef;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void ADC_Voltage_AC_ProcessData(uint32_t*, VoltageAC_ADC_NORM_Struct*);
void ADC2Phy_Voltage_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, VoltageAC_ADC_NORM_Struct* VOLTAGE_ADC_AC_IN_NORM_Sub);
void ADC_Current_AC_ProcessData(uint32_t*, CurrentAC_ADC_NORM_Struct*);
void ADC2Phy_Current_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub);
void ADC_Voltage_DC_ProcessData(uint32_t*, VoltageDC_ADC_NORM_Struct*);
void ADC2Phy_DC_Voltage_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, VoltageDC_ADC_NORM_Struct* VOLTAGE_ADC_DC_IN_NORM_Sub);
void ADC_Current_DC_ProcessData(uint32_t*, CurrentDC_ADC_NORM_Struct_t*);
void ADC2Phy_DC_Current_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, CurrentDC_ADC_NORM_Struct_t* CURRENT_ADC_DC_IN_NORM_Sub);
void DPC_ADC_Init(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,float G_Vac,float B_Vac,float G_Iac,float B_Iac,float G_Vdc,float B_Vdc,float G_Idc,float B_Idc);
#endif