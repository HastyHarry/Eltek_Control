/**
  ******************************************************************************
  * @file           : adc_convert.c
  * @brief          : manage the adc converter
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
/* Includes ------------------------------------------------------------------*/

#include "DPC_Datacollector.h"
#include "DPC_PWMConverter.h"
#include "DPC_adc_converter.h"
#include "adc.h"

/* Private variables ---------------------------------------------------------*/  
/* Private typedef -----------------------------------------------------------*/    
/* Private function prototypes -----------------------------------------------*/    
    
/**
  * @brief  ADC_Voltage_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  VOLTAGE_ADC_AC_IN_NORM_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC_Voltage_AC_ProcessData(uint32_t* p_Data_Sub, VoltageAC_ADC_NORM_Struct* VOLTAGE_ADC_AC_IN_NORM_Sub){
  VOLTAGE_ADC_AC_IN_NORM_Sub->phA=((float)(p_Data_Sub[0])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)
  VOLTAGE_ADC_AC_IN_NORM_Sub->phB=((float)(p_Data_Sub[1])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)
  VOLTAGE_ADC_AC_IN_NORM_Sub->phC=((float)(p_Data_Sub[2])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)

}
  


/**
  * @brief  ADC2Phy_Voltage_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  VOLTAGE_ADC_AC_IN_Phy_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC2Phy_Voltage_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, VoltageAC_ADC_NORM_Struct* VOLTAGE_ADC_AC_IN_NORM_Sub){
  
  float B_Vac=DPC_ADC_Conf->B_Vac;
//  float G_Vac=DPC_ADC_Conf->G_Vac;
  float invG_Vac=DPC_ADC_Conf->invG_Vac;  
  
  VOLTAGE_ADC_AC_IN_NORM_Sub->phA=((float)((int16_t)p_Data_Sub[0]-B_Vac)*(float)(invG_Vac));    
  VOLTAGE_ADC_AC_IN_NORM_Sub->phB=((float)((int16_t)p_Data_Sub[1]-B_Vac)*(float)(invG_Vac));    
  VOLTAGE_ADC_AC_IN_NORM_Sub->phC=((float)((int16_t)p_Data_Sub[2]-B_Vac)*(float)(invG_Vac));    
}


    

/**
  * @brief  ADC_Voltage_DC_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  VOLTAGE_ADC_DC_IN_NORM_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC_Voltage_DC_ProcessData(uint32_t* p_Data_Sub, VoltageDC_ADC_NORM_Struct* VOLTAGE_ADC_DC_IN_NORM_Sub){
  VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_pos=((float)(p_Data_Sub[0])/(float)(1<<11));    //(float)(1<<11)==(2.44140625e-4*2)
  VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_neg=((float)(p_Data_Sub[1])/(float)(1<<11));    //(float)(1<<11)==(2.44140625e-4*2)
}  


/**
  * @brief  ADC2Phy_DC_Voltage_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  VOLTAGE_ADC_DC_IN_NORM_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC2Phy_DC_Voltage_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, VoltageDC_ADC_NORM_Struct* VOLTAGE_ADC_DC_IN_NORM_Sub){
  
  float B_Vdc=DPC_ADC_Conf->B_Vdc;
//  float G_Vdc=DPC_ADC_Conf->G_Vdc;
  float invG_Vdc=DPC_ADC_Conf->invG_Vdc;
  
  VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_pos=((float)((int16_t)p_Data_Sub[0]-B_Vdc)*(float)(invG_Vdc)); 
  VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_neg=((float)((int16_t)p_Data_Sub[1]-100)*(float)(invG_Vdc));
  VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_tot=VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_pos+VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_neg;
  
}  

  

/**
  * @brief  ADC_Current_AC_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  CURRENT_ADC_AC_IN_NORM_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */    
void ADC_Current_AC_ProcessData(uint32_t* p_Data_Sub, CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub){
  CURRENT_ADC_AC_IN_NORM_Sub->phA=((float)(p_Data_Sub[0])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)
  CURRENT_ADC_AC_IN_NORM_Sub->phB=((float)(p_Data_Sub[1])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)
  CURRENT_ADC_AC_IN_NORM_Sub->phC=((float)(p_Data_Sub[2])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)

}




/**
  * @brief  ADC2Phy_Current_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  CURRENT_ADC_AC_IN_Phy_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC2Phy_Current_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub){
  
  float B_Iac=DPC_ADC_Conf->B_Iac;
//  float G_Iac=DPC_ADC_Conf->G_Iac;
  float invG_Iac=DPC_ADC_Conf->invG_Iac;  
  
  CURRENT_ADC_AC_IN_NORM_Sub->phA=((float)((int16_t)p_Data_Sub[0]-B_Iac)*(float)(invG_Iac));    
  CURRENT_ADC_AC_IN_NORM_Sub->phB=((float)((int16_t)p_Data_Sub[1]-B_Iac)*(float)(invG_Iac));    
  CURRENT_ADC_AC_IN_NORM_Sub->phC=((float)((int16_t)p_Data_Sub[2]-B_Iac)*(float)(invG_Iac));    
}



/**
  * @brief  ADC_Current_DC_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  CURRENT_ADC_DC_IN_NORM_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC_Current_DC_ProcessData(uint32_t* p_Data_Sub, CurrentDC_ADC_NORM_Struct_t* CURRENT_ADC_DC_IN_NORM_Sub){
  CURRENT_ADC_DC_IN_NORM_Sub->IDC_adc=((float)(p_Data_Sub[0])/(float)(1<<11)-1);    //(float)(1<<11)==(2.44140625e-4*2)
}


/**
  * @brief  ADC2Phy_DC_Current_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  CURRENT_ADC_DC_IN_NORM_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC2Phy_DC_Current_ProcessData(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,uint32_t* p_Data_Sub, CurrentDC_ADC_NORM_Struct_t* CURRENT_ADC_DC_IN_NORM_Sub){
  
  float B_Idc=DPC_ADC_Conf->B_Idc;
//  float G_Idc=DPC_ADC_Conf->G_Idc;
  float invG_Idc=DPC_ADC_Conf->invG_Idc;   
  
  CURRENT_ADC_DC_IN_NORM_Sub->IDC_adc=((float)((int16_t)p_Data_Sub[0]-B_Idc)*(float)(invG_Idc)); 
}





/**
  * @brief  DPC_ADC_Init
  * @param  DPC_ADC_Conf_TypeDef
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */    

void DPC_ADC_Init(DPC_ADC_Conf_TypeDef *DPC_ADC_Conf,float G_Vac,float B_Vac,float G_Iac,float B_Iac,float G_Vdc,float B_Vdc,float G_Idc,float B_Idc){
  
	DPC_ADC_Default_Init();

	DPC_ADC_Conf->B_Vac=B_Vac;
	DPC_ADC_Conf->G_Vac=G_Vac;
	DPC_ADC_Conf->invG_Vac=(float)(1.0/G_Vac);

	DPC_ADC_Conf->B_Vdc=B_Vdc;
	DPC_ADC_Conf->G_Vdc=G_Vdc;
	DPC_ADC_Conf->invG_Vdc=(float)(1.0/G_Vdc);

	DPC_ADC_Conf->B_Iac=B_Iac;
	DPC_ADC_Conf->G_Iac=G_Iac;
	DPC_ADC_Conf->invG_Iac=(float)(1.0/G_Iac);

	DPC_ADC_Conf->B_Idc=B_Idc;
	DPC_ADC_Conf->G_Idc=G_Idc;
	DPC_ADC_Conf->invG_Idc=(float)(1.0/G_Idc);


	DPC_ADC_Conf->DPC_ADC_Conf_Complete=SET;
  
}

/**
  * @brief  DPC_ADC_Default_Init
  * @param  None
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family
  */

void DPC_ADC_Default_Init(){

	  ADC_MultiModeTypeDef multimode = {0};
	  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
	  ADC_ChannelConfTypeDef sConfig = {0};

	  /** Common config
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.GainCompensation = 0;
	  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.NbrOfConversion = 8;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DMAContinuousRequests = ENABLE;
	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc1.Init.OversamplingMode = ENABLE;
	  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
	  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
	  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure the ADC multi-mode
	  */
	  multimode.Mode = ADC_MODE_INDEPENDENT;
	  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Analog WatchDog 1
	  */
	  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
	  AnalogWDGConfig.ITMode = DISABLE;
	  AnalogWDGConfig.HighThreshold = 0x0;
	  AnalogWDGConfig.LowThreshold = 0x0;
	  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
	  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_2;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = ADC_REGULAR_RANK_3;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_4;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = ADC_REGULAR_RANK_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = ADC_REGULAR_RANK_6;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank = ADC_REGULAR_RANK_7;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
	  sConfig.Rank = ADC_REGULAR_RANK_8;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


/**
  * @brief  Current_DC_Calc
  * @param  VOLTAGE_ADC_DC_IN_NORM_Sub DC Voltage Value in Volts
  * @param  VOLTAGE_ADC_AC_IN_NORM_Sub AC Voltage Value in Volts
  * @param  CURRENT_ADC_AC_IN_NORM_Sub AC Current Value in Amps
  *
  * @retval CURRENT_ADC_DC_IN_NORM_Sub DC Current Value in Amps
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void Current_DC_Calc(VoltageDC_ADC_NORM_Struct* VOLTAGE_ADC_DC_IN_NORM_Sub, VoltageAC_ADC_NORM_Struct* VOLTAGE_ADC_AC_IN_NORM_Sub,
			VoltageAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub, CurrentDC_ADC_NORM_Struct_t* CURRENT_ADC_DC_IN_NORM_Sub){
	float P_AC1;
	float P_AC2;
	float P_AC3;
	P_AC1 = (float) VOLTAGE_ADC_AC_IN_NORM_Sub->phA * CURRENT_ADC_AC_IN_NORM_Sub->phA;
	P_AC2 = (float) VOLTAGE_ADC_AC_IN_NORM_Sub->phB * CURRENT_ADC_AC_IN_NORM_Sub->phB;
	P_AC3 = (float) VOLTAGE_ADC_AC_IN_NORM_Sub->phC * CURRENT_ADC_AC_IN_NORM_Sub->phC;

	CURRENT_ADC_DC_IN_NORM_Sub->IDC_adc = (float)((P_AC1+P_AC2+P_AC3)/VOLTAGE_ADC_DC_IN_NORM_Sub->Vdc_tot);
}
