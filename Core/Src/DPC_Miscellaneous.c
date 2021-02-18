/**
******************************************************************************
* @file           : DPC_Miscellaneous.c
* @brief          : Miscellaneous fuction management
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
/* Includes ------------------------------------------------------------------*/

#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

/* Private includes ---------------------------------------------------------*/
#include "DPC_Miscellaneous.h"
#include "DPC_Datacollector.h"
#include "DPC_Faulterror.h"
#include "tim.h"
//#include "hrtim.h"
#include "gpio.h"




/* external variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  DPC_MISC_Check_DCLoad: Check Load in DC Autput sistems 
*         
* @param p_ADC_Data_Sub: pointer to readed output DC current value  
* @param DC_Load_Limit_sub: Converter limit data structure  
*
* @retval DPC_Load_Status_TypeDef: Load status, NO_LOAD, LOW_LOAD, ON_LOAD, 
*             OVERVOLTAGE_LOAD, OVERCURRENT_LOAD.
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/

DPC_Load_Status_TypeDef DPC_MISC_Check_DCLoad(DPC_Load_TypeDef *DPC_Load_loc,DPC_Load_Limit_TypeDef DC_Load_Limit_sub)
{

  uint16_t VDC;
  VoltageDC_ADC_Struct* DATA_VDC;  
  CurrentDC_ADC_Struct* DATA_IDC;
  
  DATA_VDC =  Read_Volt_DC(); 
  DATA_IDC = Read_Curr_DC(); 
  
  VDC=DATA_VDC->Vdc_pos+DATA_VDC->Vdc_neg;
  
  DPC_Load_Status_TypeDef Load_Status;
 
  
  if((DATA_VDC->Vdc_pos > DC_Load_Limit_sub.V_cap_Limit) || (DATA_VDC->Vdc_neg > DC_Load_Limit_sub.V_cap_Limit)){  
    DPC_PWM_OutDisable();                                                                   ///Safe: Disable PWM outputs if enabled
//    DPC_MISC_RELAY_Cntl(RELAY_GRID_ABC, RELAY_OFF);                                         ///Safe: Disconnect teh AC main
    Load_Status=OVERVOLTAGE_CAP;
    DPC_FLT_Faulterror_Set(FAULT_OVC);    
  }
  else if (VDC>DC_Load_Limit_sub.V_dc_Limit){
    DPC_PWM_OutDisable();                                                                   ///Safe: Disable PWM outputs if enabled
//    DPC_MISC_RELAY_Cntl(RELAY_GRID_ABC, RELAY_OFF);                                         ///Safe: Disconnect teh AC main
    Load_Status=OVERVOLTAGE_LOAD;
    DPC_FLT_Faulterror_Set(FAULT_OVL);
  }
  else {
    if(DATA_IDC->IDC_adc>=DC_Load_Limit_sub.I_Over_load_Threshold)
    {
      Load_Status=OVERCURRENT_LOAD;
      DPC_FLT_Faulterror_Set(FAULT_OCL);       
    }
    else if(DATA_IDC->IDC_adc>=DC_Load_Limit_sub.I_Low_load_Max_Threshold && (DPC_Load_loc->DPC_Load_Status==LOW_LOAD || DPC_Load_loc->DPC_Load_Status==NO_LOAD))
    {
      Load_Status=ON_LOAD;
    }
    else if(DATA_IDC->IDC_adc<=DC_Load_Limit_sub.I_Low_load_Min_Threshold && (DPC_Load_loc->DPC_Load_Status==ON_LOAD))
    {
      Load_Status=LOW_LOAD;
    }
    else if(DATA_IDC->IDC_adc>=DC_Load_Limit_sub.I_No_load_Max_Threshold && (DPC_Load_loc->DPC_Load_Status==NO_LOAD))
    {
      Load_Status=LOW_LOAD;
    }
    else if(DATA_IDC->IDC_adc<=DC_Load_Limit_sub.I_No_load_Min_Threshold)
    {
      Load_Status=NO_LOAD;
    }
    else
    {
      Load_Status=DPC_Load_loc->DPC_Load_Status;
    }
  }
  
  DPC_Load_loc->DPC_Load_Status=Load_Status;
  return Load_Status;
}





/**
* @brief  DPC_MISC_Check_DCLoad: Check Load in DC Autput sistems 
*         
* @param V_cap_Limit_VOLT DC voltage limit expressed in Peak voltage
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/

void DPC_MISC_DCLoad_Init(DPC_Load_Limit_TypeDef *DC_Load_Limit_sub,uint16_t V_dc_Limit_VOLT,uint16_t V_cap_Limit_VOLT,float I_dc_NO_LOAD_Limit_AMP,float I_dc_LOW_LOAD_Limit_AMP,float I_dc_OVER_LOAD_Limit_AMP,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc)
{
  
 
  uint16_t V_cap_Limit_loc;                                                                             /// Local variable to pass Output capacitors voltage limit theshold (Expressed in VOLT) 
  uint16_t V_dc_Limit_loc;                                                                              /// Local variable to pass Output voltage limit theshold (Expressed in VOLT)   
  uint16_t I_dc_NO_LOAD_Limit_loc;                                                                      /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_NO_LOAD_Delta_Limit_loc;                                                                /// 
  uint16_t I_dc_NO_LOAD_Max_Limit_loc;                                                                  /// Local variable to pass Output current Max theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_NO_LOAD_Min_Limit_loc;                                                                  /// Local variable to pass Output current Min theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_LOW_LOAD_Limit_loc;                                                                     /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_LOW_LOAD_Delta_Limit_loc;                                                                /// 
  uint16_t I_dc_LOW_LOAD_Max_Limit_loc;                                                                 /// Local variable to pass Output current Max theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_LOW_LOAD_Min_Limit_loc;                                                                 /// Local variable to pass Output current Min theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_OVER_LOAD_Limit_loc;                                                                    /// Local variable to pass Output current theshold (Expressed in AMPs)to determinate Over Load Condition
  

  
  V_cap_Limit_loc=(uint16_t)(((float)V_cap_Limit_VOLT*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);                                        /// (Vcap_limit_Threshold [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias 
  V_dc_Limit_loc=(uint16_t)(((float)V_dc_Limit_VOLT*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);                                          /// (Vdc_limit_Threshold [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias

  I_dc_NO_LOAD_Limit_loc=(uint16_t)(((float)I_dc_NO_LOAD_Limit_AMP*DPC_ADC_Conf_loc->G_Idc)+DPC_ADC_Conf_loc->B_Idc);                           /// (IDC_No_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_NO_LOAD_Delta_Limit_loc=(uint16_t)((I_dc_NO_LOAD_Limit_loc - DPC_ADC_Conf_loc->B_Idc)*((float)DPC_NO_LOAD_DELTA_CURR*0.01));                     ///
  I_dc_NO_LOAD_Max_Limit_loc=(uint16_t)(I_dc_NO_LOAD_Limit_loc + I_dc_NO_LOAD_Delta_Limit_loc);                                                 /// 
  I_dc_NO_LOAD_Min_Limit_loc=(uint16_t)(I_dc_NO_LOAD_Limit_loc - I_dc_NO_LOAD_Delta_Limit_loc);                                                 /// 

  I_dc_LOW_LOAD_Limit_loc=(uint16_t)(((float)I_dc_LOW_LOAD_Limit_AMP*DPC_ADC_Conf_loc->G_Idc)+DPC_ADC_Conf_loc->B_Idc);                         /// (IDC_Light_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_LOW_LOAD_Delta_Limit_loc=(uint16_t)((I_dc_LOW_LOAD_Limit_loc - DPC_ADC_Conf_loc->B_Idc)*((float)DPC_LOW_LOAD_DELTA_CURR*0.01));                   ///
  I_dc_LOW_LOAD_Max_Limit_loc=(uint16_t)(I_dc_LOW_LOAD_Limit_loc + I_dc_LOW_LOAD_Delta_Limit_loc);                                               /// 
  I_dc_LOW_LOAD_Min_Limit_loc=(uint16_t)(I_dc_LOW_LOAD_Limit_loc - I_dc_LOW_LOAD_Delta_Limit_loc);                                               /// 

  I_dc_OVER_LOAD_Limit_loc=(uint16_t)(((float)I_dc_OVER_LOAD_Limit_AMP*DPC_ADC_Conf_loc->G_Idc)+DPC_ADC_Conf_loc->B_Idc);                       /// (IDC_Over_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias   
  
  DC_Load_Limit_sub->V_cap_Limit=V_cap_Limit_loc;  
  DC_Load_Limit_sub->V_dc_Limit=V_dc_Limit_loc;  
  DC_Load_Limit_sub->I_No_load_Threshold=I_dc_NO_LOAD_Limit_loc;
  DC_Load_Limit_sub->I_No_load_Max_Threshold=I_dc_NO_LOAD_Max_Limit_loc;
  DC_Load_Limit_sub->I_No_load_Min_Threshold=I_dc_NO_LOAD_Min_Limit_loc;  
  DC_Load_Limit_sub->I_Low_load_Threshold=I_dc_LOW_LOAD_Limit_loc;
  DC_Load_Limit_sub->I_Low_load_Max_Threshold=I_dc_LOW_LOAD_Max_Limit_loc;
  DC_Load_Limit_sub->I_Low_load_Min_Threshold=I_dc_LOW_LOAD_Min_Limit_loc;
  DC_Load_Limit_sub->I_Over_load_Threshold=I_dc_OVER_LOAD_Limit_loc;
  
  
}




/**
* @brief  DPC_MISC_ACSource_Init: TBD
*         
* @param V_ac_Limit_VOLT AC voltage limit expressed in Peak voltage
* @param I_ac_Limit_AMP AC current limit expressed in Peak current
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/

void DPC_MISC_ACSource_Init(DPC_Source_Limit_TypeDef *AC_Source_Limit_sub,uint16_t V_ac_Limit_VOLT,uint16_t V_ac_UV_Limit_VOLT,uint16_t V_ac_UVLO_Limit_VOLT,uint16_t V_ac_Low_Limit_VOLT,uint16_t I_ac_Limit_AMP,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc)
{
 
  uint16_t V_ac_pos_Limit_loc;                                                                                                          /*!< >*/
  uint16_t V_ac_neg_Limit_loc;                                                                                                          /*!< >*/

  uint16_t V_ac_pos_UVLO_Limit_loc;                                                                                                     /*!< >*/
  uint16_t V_ac_neg_UVLO_Limit_loc;                                                                                                     /*!< >*/
  
  uint16_t V_ac_pos_UV_Limit_loc;                                                                                                       /*!< >*/
  uint16_t V_ac_neg_UV_Limit_loc;                                                                                                       /*!< >*/
  
  uint16_t V_ac_pos_Low_Limit_loc;                                                                                                      /*!< >*/
  uint16_t V_ac_neg_Low_Limit_loc;                                                                                                      /*!< >*/
  
  uint16_t I_ac_pos_Limit_loc;                                                                                                          /*!< >*/
  uint16_t I_ac_neg_Limit_loc;                                                                                                          /*!< >*/
   
  V_ac_pos_Limit_loc=(uint16_t)(((float)V_ac_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                              /*!< >*/
  V_ac_neg_Limit_loc=(uint16_t)(((float)-V_ac_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                             /*!< >*/
  
  V_ac_pos_UVLO_Limit_loc=(uint16_t)(((float)V_ac_UVLO_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                    /*!< >*/
  V_ac_neg_UVLO_Limit_loc=(uint16_t)(((float)-V_ac_UVLO_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                   /*!< >*/
  
  V_ac_pos_UV_Limit_loc=(uint16_t)(((float)V_ac_UV_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                        /*!< >*/
  V_ac_neg_UV_Limit_loc=(uint16_t)(((float)-V_ac_UV_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                       /*!< >*/
  
  V_ac_pos_Low_Limit_loc=(uint16_t)(((float)V_ac_Low_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                      /*!< >*/
  V_ac_neg_Low_Limit_loc=(uint16_t)(((float)-V_ac_Low_Limit_VOLT*DPC_ADC_Conf_loc->G_Vac)+DPC_ADC_Conf_loc->B_Vac);                     /*!< >*/
  
  I_ac_pos_Limit_loc=(uint16_t)(((float)I_ac_Limit_AMP*DPC_ADC_Conf_loc->G_Iac)+DPC_ADC_Conf_loc->B_Iac);                               /*!< >*/
  I_ac_neg_Limit_loc=(uint16_t)(((float)-I_ac_Limit_AMP*DPC_ADC_Conf_loc->G_Iac)+DPC_ADC_Conf_loc->B_Iac);                              /*!< >*/
  
  
  AC_Source_Limit_sub->V_ac_pos_Limit=V_ac_pos_Limit_loc;                                                                               /*!< >*/
  AC_Source_Limit_sub->V_ac_neg_Limit=V_ac_neg_Limit_loc;                                                                               /*!< >*/
  
  AC_Source_Limit_sub->V_ac_pos_UVLO_Limit=V_ac_pos_UVLO_Limit_loc;                                                                     /*!< >*/
  AC_Source_Limit_sub->V_ac_neg_UVLO_Limit=V_ac_neg_UVLO_Limit_loc;                                                                     /*!< >*/  
    
  AC_Source_Limit_sub->V_ac_pos_UV_Limit=V_ac_pos_UV_Limit_loc;                                                                         /*!< >*/
  AC_Source_Limit_sub->V_ac_neg_UV_Limit=V_ac_neg_UV_Limit_loc;                                                                         /*!< >*/
    
  AC_Source_Limit_sub->V_ac_pos_Low_Limit=V_ac_pos_Low_Limit_loc;                                                                       /*!< >*/
  AC_Source_Limit_sub->V_ac_neg_Low_Limit=V_ac_neg_Low_Limit_loc;                                                                       /*!< >*/
  
  AC_Source_Limit_sub->I_ac_pos_Limit=I_ac_pos_Limit_loc;
  AC_Source_Limit_sub->I_ac_neg_Limit=I_ac_neg_Limit_loc;
  
  
}


 




/**
* @brief  DPC_MISC_APPL_Timer_Init: Initialize the Application timer tu the period desidered
*         
* @param AppTIM: Timer to initialize.
* @param APPL_Freq_Desidered: Value of frequency desidered
* @retval None
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/
void DPC_MISC_APPL_Timer_Init(TIM_HandleTypeDef AppTIM, uint32_t  APPL_Freq_Desidered)
{

  uint32_t Timers_Clock;                                                                ///
  uint32_t Timers_PSC;                                                                  ///
  uint32_t Timers_ClockPSCed;                                                           ///

  Timers_PSC=(uint32_t)(READ_REG(AppTIM.Instance->PSC));                                ///
  Timers_Clock=HAL_RCC_GetPCLK2Freq();                                                  ///  
  
  Timers_ClockPSCed=(Timers_Clock/(Timers_PSC+1));                                      ///
  
  AppTIM.Init.Period = ((Timers_ClockPSCed/APPL_Freq_Desidered) - 1);                   ///

  if (HAL_TIM_Base_Init(&AppTIM) != HAL_OK){Error_Handler();}                           ///Init Task Timer  
 
} 




void DPC_MISC_Appl_Timer_Start(void)
{
  HAL_TIM_Base_Start_IT(&APPL_Tim1);  //Inizializza il Timer con IT che gestisce l'aggiornamento del duty
  HAL_TIM_Base_Start_IT(&APPL_Tim3);  //Inizializza il Timer con IT che gestisce l'aggiornamento del duty
  HAL_TIM_Base_Start_IT(&APPL_Tim2);  //Inizializza il Timer con IT che gestisce l'aggiornamento ddel display 
    
  HAL_TIM_PWM_Start(&APPL_Tim4, TIM_CHANNEL_1);   
}



///**
//  * @brief  DPC_MISC_RELAY_Cntl:
//  * @param  Relay_Typedef: Relay tu command, value accepted :RELAY_GRID_A, RELAY_GRID_B, RELAY_GRID_C,RELAY_SER_1,
//  *             RELAY_SER_2, RELAY_SER_3, RELAY_SER_4, RELAY_FAN.
//  * @param  DPC_Relay_State_TypeDef: styate to set, value accepted: RELAY_OFF, RELAY_ON.
//  * @retval None.
//  *
//  * @note Function valid for STM32G4xx microconroller family
//  */
//void DPC_MISC_RELAY_Cntl(DPC_Relay_TypeDef Relay_n, DPC_Relay_State_TypeDef bState)
//{
//#ifdef USE_RELAY_GRID_A
//    if(Relay_n & RELAY_GRID_A){
//      HAL_GPIO_WritePin(RELAY_GRID_A_PORT, RELAY_GRID_A_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_GRID_B
//    if(Relay_n & RELAY_GRID_B){
//      HAL_GPIO_WritePin(RELAY_GRID_B_PORT, RELAY_GRID_B_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_GRID_C
//    if(Relay_n & RELAY_GRID_C){
//      HAL_GPIO_WritePin(RELAY_GRID_C_PORT, RELAY_GRID_C_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_GRID_N
//    if(Relay_n & RELAY_GRID_N){
//      HAL_GPIO_WritePin(RELAY_GRID_N_PORT, RELAY_GRID_N_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_SER_1
//    if(Relay_n & RELAY_SER_1){
//      HAL_GPIO_WritePin(RELAY_SER_1_PORT, RELAY_SER_1_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_SER_2
//    if(Relay_n & RELAY_SER_2){
//      HAL_GPIO_WritePin(RELAY_SER_2_PORT, RELAY_SER_2_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_SER_3
//    if(Relay_n & RELAY_SER_3){
//      HAL_GPIO_WritePin(RELAY_SER_3_PORT, RELAY_SER_3_PIN, bState);
//    }
//#endif
//#ifdef USE_RELAY_FAN
//    if(Relay_n & RELAY_FAN){
//      HAL_GPIO_WritePin(RELAY_FAN_PORT, RELAY_FAN_PIN, bState);
//    }
//#endif
//}




/**
* @brief  DPC_MISC_BLED_Set: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/



void DPC_MISC_BLED_Set(TIM_HandleTypeDef *htim_bled,uint32_t TIM_CHANNEL_BLED,DPC_BLED_TypeDef State_BLED){
  switch ( State_BLED){
  case BLED_Idle:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xB000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    break;
  case BLED_StartUp_inrush:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xE000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    break;
  case BLED_Fault:
    HAL_Delay(600);
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xFFFF);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    HAL_Delay(600);
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xB000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    break;
  case BLED_Error:
    HAL_Delay(100);
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xFFFF);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    HAL_Delay(100);
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0x0000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);       
    break;
  case BLED_Run:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xFFFF);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);         
    break; 
  case BLED_StartUp_burst: 
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0x0FFF);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);       
    break; 
  case BLED_Stop:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xD000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);             
    break;
  case BLED_Debug:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xA000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);             
    break; 
  case BLED_Wait:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, 0xC000);  // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);             
    break;     
  }  
}





/**
* @brief  DPC_MISC_AC_SOURCE_Plugged: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/

DPC_Status_Plug_ACSource_TypeDef DPC_MISC_AC_SOURCE_Plugged(DPC_Source_Limit_TypeDef AC_Source_Limit_sub){
  
  DPC_Status_Plug_ACSource_TypeDef Status_Plug_ACSource; 
  
  VoltageAC_ADC_Struct* DATA_VAC;                                                                       /*!< >*/

  uint16_t V_ac_pos_Plug_Limit_local;                                                                   /*!< >*/
  uint16_t V_ac_neg_Plug_Limit_local;                                                                   /*!< >*/
    
  V_ac_pos_Plug_Limit_local=AC_Source_Limit_sub.V_ac_pos_Low_Limit;                                     /*!< >*/
  V_ac_neg_Plug_Limit_local=AC_Source_Limit_sub.V_ac_neg_Low_Limit;                                     /*!< >*/
  
  DATA_VAC = Read_GRID();                                                                               /*!< >*/
  
      if((DATA_VAC->phA < V_ac_pos_Plug_Limit_local) && (DATA_VAC->phA > V_ac_neg_Plug_Limit_local)){ 
        Status_Plug_ACSource=NO_Plug_ACSource;
      }
      else
      {
        Status_Plug_ACSource=OK_Plug_ACSource;        
      }
   return Status_Plug_ACSource;
  
}



/**
* @brief  DPC_MISC_CHECK_AC_SOURCE: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/

DPC_Source_Status_TypeDef DPC_MISC_CHECK_AC_SOURCE(DPC_Source_TypeDef  *AC_Source_sub,DPC_Source_Limit_TypeDef AC_Source_Limit_sub,float Theta){
  
  DPC_Source_Status_TypeDef Status_Source = AC_Source_sub->Status_Source; 
  
  float Theta_start = 0.08;                                                                             /*!< Approximation of 2Pi to define the SIN near to zero crossing>*/
  float Theta_stop = 6.20;                                                                              /*!< Approximation of 2Pi to define the SIN near to zero crossing>*/
  

  VoltageAC_ADC_Struct* DATA_VAC;                                                                       /*!< >*/
  CurrentAC_ADC_Struct* DATA_IAC;                                                                       /*!< >*/
  
  uint16_t V_ac_pos_Limit_local;                                                                        /*!< >*/
  uint16_t V_ac_neg_Limit_local;                                                                        /*!< >*/
  
  uint16_t V_ac_pos_UVLO_Limit_local;                                                                   /*!< >*/
  uint16_t V_ac_neg_UVLO_Limit_local;                                                                   /*!< >*/
  
  uint16_t V_ac_pos_UV_Limit_local;                                                                     /*!< >*/
  uint16_t V_ac_neg_UV_Limit_local;                                                                     /*!< >*/
  
  uint16_t V_ac_pos_Low_Limit_local;                                                                    /*!< >*/
  uint16_t V_ac_neg_Low_Limit_local;                                                                    /*!< >*/
  
  
  uint16_t I_ac_pos_Limit_local;                                                                        /*!< >*/
  uint16_t I_ac_neg_Limit_local;                                                                        /*!< >*/
  
  
  V_ac_pos_Limit_local=AC_Source_Limit_sub.V_ac_pos_Limit;                                              /*!< >*/
  V_ac_neg_Limit_local=AC_Source_Limit_sub.V_ac_neg_Limit;                                              /*!< >*/
  
  V_ac_pos_UVLO_Limit_local=AC_Source_Limit_sub.V_ac_pos_UVLO_Limit;                                    /*!< >*/
  V_ac_neg_UVLO_Limit_local=AC_Source_Limit_sub.V_ac_neg_UVLO_Limit;                                    /*!< >*/
  
  V_ac_pos_UV_Limit_local=AC_Source_Limit_sub.V_ac_pos_UV_Limit;                                        /*!< >*/
  V_ac_neg_UV_Limit_local=AC_Source_Limit_sub.V_ac_neg_UV_Limit;                                        /*!< >*/
  
  V_ac_pos_Low_Limit_local=AC_Source_Limit_sub.V_ac_pos_Low_Limit;                                      /*!< >*/
  V_ac_neg_Low_Limit_local=AC_Source_Limit_sub.V_ac_neg_Low_Limit;                                      /*!< >*/
  
  I_ac_pos_Limit_local=AC_Source_Limit_sub.I_ac_pos_Limit;                                              /*!< >*/
  I_ac_neg_Limit_local=AC_Source_Limit_sub.I_ac_neg_Limit;                                              /*!< >*/
  
  
  
  DATA_VAC = Read_GRID();                                                                               /*!< >*/
  DATA_IAC = Read_Curr_GRID();                                                                          /*!< >*/
  
  
	if((DATA_IAC->phA > I_ac_pos_Limit_local) || (DATA_IAC->phA < I_ac_neg_Limit_local))
	{
		Status_Source=OVERCURRENT_SOURCE;
		//DPC_FLT_Faulterror_Set(FAULT_OCS);
	}

	if((DATA_IAC->phB > I_ac_pos_Limit_local) || (DATA_IAC->phB < I_ac_neg_Limit_local))
	{
		Status_Source=OVERCURRENT_SOURCE;
		//DPC_FLT_Faulterror_Set(FAULT_OCS);
	}
  
  
//  if((DATA_VAC->phA > V_ac_pos_Limit_local) || (DATA_VAC->phA < V_ac_neg_Limit_local))
//  {  
//    Status_Source=OVERVOLTAGE_SOURCE;
//    DPC_FLT_Faulterror_Set(FAULT_OVS);  
//  }
//  else{  
    
    if(Theta<=Theta_start){    
      /// Reset pk max & pk min of AC Voltage    
      AC_Source_sub->V_ac_pk_pos_local=0;
      AC_Source_sub->V_ac_pk_neg_local=1<<12;
      /// Reset pk max & pk min of AC Current  
      AC_Source_sub->I_ac_pk_pos_local=0;
      AC_Source_sub->I_ac_pk_neg_local=1<<12;
      Status_Source=AC_Source_sub->Status_Source;
    }
    else if(Theta>Theta_start && Theta<Theta_stop){
      /// Determinate pk max & pk min of AC Voltage
      if(AC_Source_sub->V_ac_pk_pos_local<DATA_VAC->phA){
        AC_Source_sub->V_ac_pk_pos_local=DATA_VAC->phA;
      }
      if(AC_Source_sub->V_ac_pk_neg_local>DATA_VAC->phA){
        AC_Source_sub->V_ac_pk_neg_local=DATA_VAC->phA;
      }
      /// Determinate pk max & pk min of AC Current
      
      if(AC_Source_sub->I_ac_pk_pos_local<DATA_IAC->phA){
        AC_Source_sub->I_ac_pk_pos_local=DATA_IAC->phA;
      }
      if(AC_Source_sub->I_ac_pk_neg_local>DATA_IAC->phA){
        AC_Source_sub->I_ac_pk_neg_local=DATA_IAC->phA;
      }
      
    }
    else if(Theta>=Theta_stop){
      
      if((AC_Source_sub->V_ac_pk_pos_local == 0) || (AC_Source_sub->V_ac_pk_neg_local == 1<<12)){ 
        Status_Source=WAIT_SOURCE;
      }
      else{
        if((AC_Source_sub->V_ac_pk_pos_local > V_ac_pos_Limit_local) && (AC_Source_sub->V_ac_pk_neg_local < V_ac_neg_Limit_local)){
          Status_Source=OVERVOLTAGE_SOURCE;
          DPC_FLT_Faulterror_Set(FAULT_OVS);
        } 
        else if((AC_Source_sub->V_ac_pk_pos_local < V_ac_pos_Low_Limit_local) && (AC_Source_sub->V_ac_pk_neg_local > V_ac_neg_Low_Limit_local)){
          Status_Source=NO_SOURCE;
          DPC_FLT_Faulterror_Set(ERROR_AC_OFF);
        }
        else if((AC_Source_sub->V_ac_pk_pos_local < V_ac_pos_UV_Limit_local) && (AC_Source_sub->V_ac_pk_neg_local > V_ac_neg_UV_Limit_local)){
          Status_Source=UV_SOURCE;
          DPC_FLT_Faulterror_Set(ERROR_AC_UV); 
        }      
        else {
          Status_Source=OK_SOURCE;
          if((AC_Source_sub->V_ac_pk_pos_local < V_ac_pos_UVLO_Limit_local) && (AC_Source_sub->V_ac_pk_neg_local > V_ac_neg_UVLO_Limit_local)){
            DPC_FLT_Faulterror_Set(ERROR_AC_UVLO);
          }            
        }
      }    
    }
//  }
  
  
  //  if("PLL_OK"){
  
  //  }
  //  else if ("PLL_KO"){
  //  if((DATA_VAC->phA > V_ac_pos_Limit_local) || (DATA_VAC->phA < V_ac_neg_Limit_local)){  
  //    Status_Source=OVERVOLTAGE_SOURCE;
  //  }
  //  else if((DATA_VAC->phA < V_ac_pos_Low_Limit_local) && (DATA_VAC->phA > V_ac_neg_Low_Limit_local)){
  //    Status_Source=NO_SOURCE;
  //  }
  //  else {
  //    
  //    if((DATA_IAC->phA > I_ac_pos_Limit_local) || (DATA_IAC->phA < I_ac_neg_Limit_local))
  //    {
  //      Status_Source=OVERCURRENT_SOURCE;
  //    }
  //    Status_Source=OK_SOURCE;   
  //  }    
  //  }
  
  AC_Source_sub->Status_Source=Status_Source;
  return Status_Source;
  
}




/**
* @brief  DPC_DAC_Init: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/
  void DPC_DAC_Init(DAC_Channel_STRUCT *DAC_CH_Sub, uint8_t DAC_CH1_INIT_loc, uint8_t DAC_CH2_INIT_loc, uint8_t DAC_CH3_INIT_loc, uint16_t DAC_G_CH1_Init_loc,uint16_t DAC_G_CH2_Init_loc , uint16_t DAC_G_CH3_Init_loc, uint16_t DAC_B_CH1_Init_loc,uint16_t DAC_B_CH2_Init_loc , uint16_t DAC_B_CH3_Init_loc){
  
  DAC_CH_Sub->CH1=DAC_CH1_INIT_loc;
  DAC_CH_Sub->CH2=DAC_CH2_INIT_loc;
  DAC_CH_Sub->CH3=DAC_CH3_INIT_loc;
  DAC_CH_Sub->Gain_CH1=DAC_G_CH1_Init_loc;
  DAC_CH_Sub->Gain_CH2=DAC_G_CH2_Init_loc;
  DAC_CH_Sub->Gain_CH3=DAC_G_CH3_Init_loc;
  DAC_CH_Sub->Bias_CH1=DAC_G_CH1_Init_loc;
  DAC_CH_Sub->Bias_CH2=DAC_G_CH2_Init_loc;
  DAC_CH_Sub->Bias_CH3=DAC_G_CH3_Init_loc;  
  
}


