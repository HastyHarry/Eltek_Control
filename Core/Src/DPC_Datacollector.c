/**
  ******************************************************************************
  * @file           : Data.c
  * @brief          : Data Collectiona and access interfaces
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
#include "adc.h"
#include "DPC_PWMConverter.h"
    

/* Private variables ---------------------------------------------------------*/
float DATA_theta_PLL;
VoltageAC_ADC_Struct VAC_ADC;
CurrentAC_ADC_Struct IAC_ADC;
VoltageDC_ADC_Struct VDC_ADC;
CurrentDC_ADC_Struct IDC_ADC;
  //Tensioni ABC trasformate in DQO
TRANSFORM_QDO_t DATA_VOLT_ClarkePark;
 //Correnti ABC trasformate in DQO
TRANSFORM_QDO_t DATA_CURR_ClarkePark;
RAW_ADC_UNION RAW_ADC;
//External temoerature
uint16_t T_ext;
uint16_t T_int;

 
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Read the Voltage data dtructure
  * @param  None
  *         
  * @retval Voltage data structure Address
  *
  * @note Function valid for STM32G4xx microconroller family  
  */  
VoltageAC_ADC_Struct* DATA_Vabc_Read(void)                                
{
return &VAC_ADC;
}


/**
  * @brief  write the aut of phase angle
  * @param  Theta (Radians)
  *         
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void DATA_Write_Theta_PLL(float Theta)                             
{

  DATA_theta_PLL=Theta;
}


/**
  * @brief  Read the aut of phase angle
  * @param  None
  *         
  * @retval DATA_theta_PLL (radians)
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
float DATA_Read_Theta_PLL(void)
{
  
  return DATA_theta_PLL;
}


/**
  * @brief  Write the Clark Park Transform result (current) in the database 
  * @param  Clark Park result structure
  *         
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
void DATA_CURR_Write_ClarkePark(TRANSFORM_QDO_t Results_ClarkePark)
                                
 {
                                
DATA_CURR_ClarkePark.axd=Results_ClarkePark.axd;
DATA_CURR_ClarkePark.axq=Results_ClarkePark.axq;
DATA_CURR_ClarkePark.axo=Results_ClarkePark.axo;
}


/**
  * @brief  Write the Clark Park Transform result (voltage) in the database 
  * @param  Clark Park result structure
  *         
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
void DATA_VOLT_Write_ClarkePark(TRANSFORM_QDO_t Results_ClarkePark)
                                
 {
                                
DATA_VOLT_ClarkePark.axd=Results_ClarkePark.axd;
DATA_VOLT_ClarkePark.axq=Results_ClarkePark.axq;
DATA_VOLT_ClarkePark.axo=Results_ClarkePark.axo;
}


/**
  * @brief  Read from the database the Clark Park Transform value (current) 
  * @param  None
  *         
  * @retval Clark Park result structure
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
TRANSFORM_QDO_t DATA_CURR_Read_ClarkePark(void)
{
  
  return DATA_CURR_ClarkePark;
}

/**
  * @brief  Read from the database the Clark Park Transform value (Voltage) 
  * @param  None
  *         
  * @retval Clark Park result structure
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
TRANSFORM_QDO_t DATA_VOLT_Read_ClarkePark(void)
{
  
  return DATA_VOLT_ClarkePark;
}



/**
  * @brief  Init the ADC DMA 
  * @param  None
  *         
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void InitDMA_ADC_CONV(void){
HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&RAW_ADC,9);  //HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* p_ADC1_Data, uint32_t Length)
}



/**
  * @brief  Read the ADC data raw 
  * @param  None
  *         
  * @retval RAW_ADC_UNION, pointer to Data struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
RAW_ADC_UNION* DATA_Read_ADC_Raw(void){
  
  return &RAW_ADC; 
  
}


//#ifdef ADC_GRID

/**
* @brief  Acquisition Data From a ADC with DMA.
* @param  p_ADC1_Data.
*         Pointer to ADC Data struct:
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data,uint32_t* p_ADC2_Data)                            
{   

	float temp_IA_f;
	float temp_IB_f;
	float temp_IC_f;

	uint32_t temp_IA_u;
	uint32_t temp_IB_u;
	uint32_t temp_IC_u;

//#ifdef STDES_VIENNARECT
//  //AC Side Current
//  IAC_ADC.phA=p_ADC1_Data[6];             ///Phase A Current AC Sensing
//  IAC_ADC.phB=p_ADC1_Data[5];             ///Phase B Current AC Sensing
//  IAC_ADC.phC=p_ADC1_Data[4];             ///Phase C Current AC Sensing
//  // DC Side Current
//  IDC_ADC.IDC_adc=p_ADC1_Data[2];         ///DC Current ADC sensing
//  //AC Side Voltage
//  VAC_ADC.phA=p_ADC1_Data[3];             ///Phase A Voltage AC Sensing
//  VAC_ADC.phB=p_ADC2_Data[0];             ///Phase B Voltage AC Sensing
//  VAC_ADC.phC=p_ADC2_Data[1];             ///Phase C Voltage AC Sensing
//  //MCU TEMPERATURE
//  T_int=p_ADC1_Data[7];                   ///MCU Temperature ADC Sensing
//  //DC Side Voltage
//  VDC_ADC.Vdc_pos=p_ADC1_Data[1];         ///High Side DC Voltage ADC sensing
//  VDC_ADC.Vdc_neg=p_ADC1_Data[0];         ///Low Side DC Voltage ADC sensing
//  //EXTERNAL TEMPERATURE
//  T_ext=p_ADC2_Data[2];                   ///EXT Temperature ADC Sensing
//#elif STDES_PFCBIDIR
//  T_ext=p_ADC2_Data[2];//////
//  T_int=p_ADC1_Data[7];//////
//  //AC Side Voltage
//  VAC_ADC.phA=p_ADC1_Data[4];
//  VAC_ADC.phB=p_ADC1_Data[5];
//  VAC_ADC.phC=p_ADC1_Data[6];
//  //AC Side Current
//  IAC_ADC.phA=p_ADC1_Data[0];
//  IAC_ADC.phB=p_ADC1_Data[1];
//  IAC_ADC.phC=p_ADC1_Data[2];
//  // DC Side Current
//  IDC_ADC.IDC_adc=p_ADC1_Data[3];
//  //VDC
//  VDC_ADC.Vdc_pos=p_ADC2_Data[0];
//  VDC_ADC.Vdc_neg=p_ADC2_Data[1];
//#elif STDES_PFCBIDIR_REV2


//T_ext=p_ADC2_Data[2];//////
  T_int=p_ADC1_Data[7];//////
  //AC Side Voltage
  VAC_ADC.phA=p_ADC1_Data[2];
  VAC_ADC.phB=p_ADC1_Data[3];
  VAC_ADC.phC=p_ADC1_Data[4];
  //AC Side Current                           

  //Simulating Current
  if (p_ADC1_Data[2]>B_VAC){
	  temp_IA_u = p_ADC1_Data[2]-B_VAC;
	  temp_IA_f = ((float)temp_IA_u * 0.3);
	  IAC_ADC.phA = (uint32_t)temp_IA_f + B_IAC;
  }
  else {
	  temp_IA_u = B_VAC -  p_ADC1_Data[2];
	  temp_IA_f = ((float)temp_IA_u * 0.2);
	  IAC_ADC.phA =  B_IAC - (uint32_t)temp_IA_f;
  }

  if (p_ADC1_Data[3]>B_VAC){
	  temp_IB_u = p_ADC1_Data[3]-B_VAC;
	  temp_IB_f = ((float)temp_IB_u * 0.5);
	  IAC_ADC.phB = (uint32_t)temp_IB_f + B_IAC;
  }
  else {
	  temp_IB_u = B_VAC -  p_ADC1_Data[3];
	  temp_IB_f = ((float)temp_IB_u * 0.5);
	  IAC_ADC.phB =  B_IAC - (uint32_t)temp_IB_f;
  }

  IAC_ADC.phC = B_IAC - ((IAC_ADC.phA - B_IAC) + (IAC_ADC.phB - B_IAC));


//  IAC_ADC.phA=p_ADC1_Data[0];
//  IAC_ADC.phB=p_ADC1_Data[1];
//  IAC_ADC.phC = B_IAC - ((IAC_ADC.phA - B_IAC) + (IAC_ADC.phB - B_IAC));


  //IAC_ADC.phC=p_ADC1_Data[];
  // DC Side Current
  //IDC_ADC.IDC_adc=p_ADC1_Data[3];
  //VDC                         
  VDC_ADC.Vdc_pos=p_ADC1_Data[5];
  VDC_ADC.Vdc_neg=4096-p_ADC1_Data[6];
//#elif NUCLEO_3TTC
//  //AC Side
//  VAC_ADC.phA=p_ADC1_Data[0];
//  VAC_ADC.phB=p_ADC1_Data[1];
//  VAC_ADC.phC=p_ADC1_Data[2];
//  IAC_ADC.phA=p_ADC1_Data[3];
//  IAC_ADC.phB=p_ADC1_Data[4];
//  IAC_ADC.phC=p_ADC1_Data[5];
//  // DC Side
//  VDC_ADC.Vdc_pos=p_ADC1_Data[6];
//  VDC_ADC.Vdc_neg=p_ADC1_Data[7];
//  IDC_ADC.Idc_adc=p_ADC1_Data[8];
//#else
//  SELECT DEFINE
//#endif
} 
//#endif

void DATA_Acquisition_from_LUT(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *RAMP_DATA)
{

  //T_ext=p_ADC2_Data[2];//////
  //T_int=p_ADC1_Data[7];//////
  //AC Side Voltage
  VAC_ADC.phA=(LUT_CONVERTER_sub->LUTsinefloat[RAMP_DATA->rampA]+1)*2048;
  VAC_ADC.phB=(LUT_CONVERTER_sub->LUTsinefloat[RAMP_DATA->rampB]+1)*2048;
  VAC_ADC.phC=(LUT_CONVERTER_sub->LUTsinefloat[RAMP_DATA->rampC]+1)*2048;
  //AC Side Current
  //IAC_ADC.phA=p_ADC1_Data[0];
  //IAC_ADC.phB=p_ADC1_Data[1];
  //IAC_ADC.phC=p_ADC1_Data[2];
  // DC Side Current
  //IDC_ADC.IDC_adc=p_ADC1_Data[3];
  //VDC
  //VDC_ADC.Vdc_pos=p_ADC2_Data[0];
  //VDC_ADC.Vdc_neg=p_ADC2_Data[1];
}


/**
  * @brief  Read the Grid voltage.
  * @param  None.
  * 
  * @retval VAC_ADC, pointer to VoltageAC_ADC struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
VoltageAC_ADC_Struct* Read_GRID(void){
  return &VAC_ADC;
}



/**
  * @brief  Read the Grid Current.
  * @param  None.
  * 
  * @retval IAC_ADC, pointer to CurrentAC_ADC struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
CurrentAC_ADC_Struct* Read_Curr_GRID(void){
  return &IAC_ADC;
}



/**
  * @brief  Read the DC Current.
  * @param  None.
  * 
  * @retval IAC_ADC, pointer to CurrentDC_ADC struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
CurrentDC_ADC_Struct* Read_Curr_DC(void){
  return &IDC_ADC;
}

/**
  * @brief  Read the DC Voltage.
  * @param  None.
  * 
  * @retval VDC_ADC, pointer to VoltageDC_ADC struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
VoltageDC_ADC_Struct* Read_Volt_DC(void){
  return &VDC_ADC;
}



