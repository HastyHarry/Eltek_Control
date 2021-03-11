/**
  ******************************************************************************
  * @file           : DPC_Loopctrl.c
  * @brief          : Loop Control  Module
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
#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif

#include "DPC_Pid.h"
#include "DPC_PLL.h"
#include "main.h"
#include "DPC_Transforms.h"
#include "DPC_Loopctrl.h"
#include "tim.h"



/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Current_Decoupling_Control: Current_Decoupling_Control .
  * @param  pCDC_sub:
  * @param  pPI_ID_CURR_CTRL_sub:
  * @param  pPI_IQ_CURR_CTRL_sub:
  * @param  pVd_ctrl_sub: d-axis Vcontrol CDC output term
  * @param  pVq_ctrl_sub: q-axis Vcontrol CDC output term
  *
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void Current_Decoupling_Control(CDC_Struct *pCDC_sub,PI_STRUCT_t *pPI_ID_CURR_CTRL_sub, PI_STRUCT_t *pPI_IQ_CURR_CTRL_sub ,float *pVd_ctrl_FF_sub, float *pVq_ctrl_FF_sub){

float fomegagrid_sub=pCDC_sub->omegagrid;                                                       ///Omega grid value expressed in Hz - Related to decoupled terms
float fInductor_sub=pCDC_sub->Inductor;                                                         ///Inductor value expressed in H - Related to decoupled terms
float fId_ref_sub=pCDC_sub->Id_ref;                                                             ///Local variable - d-axis current referance
float fIq_ref_sub=pCDC_sub->Iq_ref;                                                             ///Local variable - q-axis current referance
float fId_feed_sub=pCDC_sub->Id_feed;                                                           ///Local variable - d-axis current feedback
float fIq_feed_sub=pCDC_sub->Iq_feed;                                                           ///Local variable - q-axis current feedback
float fVd_Curr_Ctrl_sub;                                                                        ///Local variable - d-axis PI output
float fVq_Curr_Ctrl_sub;                                                                        ///Local variable - q-axis PI output
float fVd_Decoupling_sub;                                                                       ///Local variable - d-axis decoupling term
float fVq_Decoupling_sub;                                                                       ///Local variable - d-axis decoupling term
//float *pVd_ctrl_FF_sub;                                                                          ///Local variable - d-axis FeefForward term
//float *pVq_ctrl_FF_sub;                                                                          ///Local variable - d-axis FeefForward term
float fVdc_sub=pCDC_sub->Vdc_feed;                                                              ///Local variable - Vdc feedback

  fVd_Curr_Ctrl_sub=PI(fId_ref_sub, fId_feed_sub , pPI_ID_CURR_CTRL_sub);                       ///d-axis PI regulator block
  fVq_Curr_Ctrl_sub=PI(fIq_ref_sub, fIq_feed_sub , pPI_IQ_CURR_CTRL_sub);                       ///q-axis PI regulator block

  if(pCDC_sub->Decoupling_Enable==SET)
  {
  pCDC_sub->Vd_Decoupling=fVd_Decoupling_sub=fIq_feed_sub*fomegagrid_sub*fInductor_sub;         ///d-axis decoupling term block
  pCDC_sub->Vq_Decoupling=fVq_Decoupling_sub=fId_feed_sub*fomegagrid_sub*fInductor_sub;         ///q-axis decoupling term block
  pCDC_sub->Vd_Curr_Ctrl=(fVd_Curr_Ctrl_sub-fVd_Decoupling_sub);                                ///d-axis decoupling term applied
  pCDC_sub->Vq_Curr_Ctrl=(fVq_Curr_Ctrl_sub+fVq_Decoupling_sub);                                ///q-axis decoupling term applied
  }
  else
  {
  pCDC_sub->Vd_Curr_Ctrl=fVd_Curr_Ctrl_sub;                                                    ///d-axis decoupling term bypassed
  pCDC_sub->Vq_Curr_Ctrl=fVq_Curr_Ctrl_sub;                                                    ///q-axis decoupling term bypassed
  }



  if(pCDC_sub->FF_Enable==SET)
  {
  FeedForward_Control(pCDC_sub,pVd_ctrl_FF_sub,pVq_ctrl_FF_sub);                                ///dq-axis Feed_Forward applied
  }
  else
  {
    *pVd_ctrl_FF_sub=pCDC_sub->Vd_Curr_Ctrl;                                                    ///d-axis Feed_Forward bypassed
    *pVq_ctrl_FF_sub=pCDC_sub->Vq_Curr_Ctrl;                                                    ///q-axis Feed_Forward bypassed
  }

  if(pCDC_sub->VDC_FF_Enable==SET)
  {
    *pVd_ctrl_FF_sub=*pVq_ctrl_FF_sub/fVdc_sub;                                                 ///d-axis - Vdc Feed_Forward applied
    *pVq_ctrl_FF_sub=*pVq_ctrl_FF_sub/fVdc_sub;                                                 ///q-axis - Vdc Feed_Forward applied
  }
  else
  {
    *pVd_ctrl_FF_sub=pCDC_sub->Vd_Curr_Ctrl;                                                    ///d-axis - Vdc Feed_Forward bypassed
    *pVq_ctrl_FF_sub=pCDC_sub->Vq_Curr_Ctrl;                                                    ///q-axis - Vdc Feed_Forward bypassed
  }


  *pVd_ctrl_FF_sub=*pVd_ctrl_FF_sub;                                                               ///d-axis Vcontrol CDC output term
  *pVq_ctrl_FF_sub=*pVq_ctrl_FF_sub;                                                               ///q-axis Vcontrol CDC output term

}






/**
  * @brief  Current_Decoupling_Control_FeedForward_Term: Current_Decoupling_Control_FeedForward_Term .
  * @param  pCDC_sub:
  * @param  pVd_ctrl_FF_sub:
  * @param  pVq_ctrl_FF_sub:
  *
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family
  */

void FeedForward_Control(CDC_Struct *pCDC_sub,float *pVd_ctrl_FF_sub, float *pVq_ctrl_FF_sub)
{
float fVd_feed_sub=pCDC_sub->Vd_feed;
float fVq_feed_sub=pCDC_sub->Vq_feed;
float fVd_ctrl_sub=pCDC_sub->Vd_Curr_Ctrl;
float fVq_ctrl_sub=pCDC_sub->Vq_Curr_Ctrl;


pCDC_sub->Vd_ctrl_FF=fVd_feed_sub-fVd_ctrl_sub;
pCDC_sub->Vq_ctrl_FF=fVq_feed_sub-fVq_ctrl_sub;

  *pVd_ctrl_FF_sub=pCDC_sub->Vd_ctrl_FF;
  *pVq_ctrl_FF_sub=pCDC_sub->Vq_ctrl_FF;
}



/**
  * @brief  Voltage_Control: Voltage_Control .
  * @param  VOLTAGECTRL_sub:
  * @param  PI_VDC_CTRL_sub:
  * @param  Id_ctrl_sub:
  *
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void Voltage_Control(VOLTAGECTRL_Struct *VOLTAGECTRL_sub,PI_STRUCT_t *PI_VDC_CTRL_sub, float *Id_ctrl_sub){
  float Vdc_ref_sub=VOLTAGECTRL_sub->Vdc_ref;
  float Vdc_feed_sub=VOLTAGECTRL_sub->Vdc_feed;

  PI(Vdc_ref_sub, Vdc_feed_sub , PI_VDC_CTRL_sub);
  VOLTAGECTRL_sub->Id_ctrl=PI_VDC_CTRL_sub->PIout_sat;
  *Id_ctrl_sub=PI_VDC_CTRL_sub->PIout_sat;
}






/**
  * @brief  DPC_LPCNTRL_PFC_Mode_Reset:
  *
  * @retval null
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void DPC_LPCNTRL_PFC_Mode_Reset(PI_STRUCT_t *PI_VDC_CTRL, CDC_Struct *CDC)
{
        PI_VDC_CTRL->resetPI=SET;                                   ///Mantein Not used regulator in reset mode
        CDC->pPI_ID_CURR_CTRL.resetPI=SET;                          ///Mantein Not used regulator in reset mode
        CDC->pPI_IQ_CURR_CTRL.resetPI=SET;                          ///Mantein Not used regulator in reset mode
}


/**
  * @brief  DPC_LPCNTRL_PFC_Mode: CORE of control loop (VOLTAGE LOOP or CURRENT LOOP or OPEN LOOP)
  * @param  pPFC_CTRL_loc: TBD
  * @param  PI_VDC_CTRL: TBD
  * @param  VOLTAGECTRL: TBD
  * @param  CDC: TBD
  * @param  V_DQO_CTRL: TBD
  * @param  Current_qdo: TBD
  * @param  Voltage_qdo: TBD
  *
  * @retval null
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void DPC_LPCNTRL_PFC_Mode(PFC_CTRL_t *pPFC_CTRL_loc, PI_STRUCT_t *PI_VDC_CTRL, VOLTAGECTRL_Struct *VOLTAGECTRL, CDC_Struct *CDC,TRANSFORM_QDO_t *V_DQO_CTRL,TRANSFORM_QDO_t *Current_qdo,TRANSFORM_QDO_t *Voltage_qdo,VoltageDC_ADC_NORM_Struct *VOLTAGE_ADC_DC_IN_PHY)
{

  float Id_ctrl_sub=0;
  float Vd_ctrl_FF=0;
  float Vq_ctrl_FF=0;



  pPFC_CTRL_loc->VOLTAGECTRL=*VOLTAGECTRL;
  pPFC_CTRL_loc->CDC=*CDC;

   if(pPFC_CTRL_loc->PFC_CTRL_State==VOLTAGE_LOOP) /// Voltage and Current control closed
  {
    PI_VDC_CTRL->resetPI=pPFC_CTRL_loc->VdcCTRL_Reset;          ///Release PI accumulator
    VOLTAGECTRL->Vdc_ref=pPFC_CTRL_loc->PFC_VDC_Ref;
    VOLTAGECTRL->Vdc_feed=VOLTAGE_ADC_DC_IN_PHY->Vdc_tot;
    CDC->pPI_ID_CURR_CTRL.resetPI=pPFC_CTRL_loc->CDC_Reset;     ///Release PI accumulator
    CDC->pPI_IQ_CURR_CTRL.resetPI=pPFC_CTRL_loc->CDC_Reset;     ///Release PI accumulator
    CDC->pPI_IQ_CURR_CTRL.resetPI=pPFC_CTRL_loc->CDC_Reset;     ///Release PI accumulator

    CDC->Id_feed=Current_qdo->axd;                              ///Pass dq current feedback to CDC struct (d-axis)
    CDC->Iq_feed=Current_qdo->axq;                              ///Pass dq current feedback to CDC struct (q-axis)
    CDC->Vd_feed=Voltage_qdo->axd;                              ///Pass dq voltage feedback to CDC struct (d-axis)
    CDC->Vq_feed=Voltage_qdo->axq;                              ///Pass dq voltage feedback to CDC struct (q-axis)
    CDC->Vdc_feed=VOLTAGE_ADC_DC_IN_PHY->Vdc_tot;               ///Pass DC voltage feedback to CDC struct

    Voltage_Control(VOLTAGECTRL, PI_VDC_CTRL,&Id_ctrl_sub);
    CDC->Id_ref=Id_ctrl_sub;
    CDC->Iq_ref=0;
    Current_Decoupling_Control(CDC,&CDC->pPI_ID_CURR_CTRL,&CDC->pPI_IQ_CURR_CTRL, &Vd_ctrl_FF,&Vq_ctrl_FF);

    V_DQO_CTRL->axd=Vd_ctrl_FF;
    V_DQO_CTRL->axq=Vq_ctrl_FF;
    V_DQO_CTRL->axo=0;
  }
  else if(pPFC_CTRL_loc->PFC_CTRL_State==CURRENT_LOOP) /// Only Current control closed
  {
    PI_VDC_CTRL->resetPI=SET;                                   ///Mantein Not used regulator in reset mode
    CDC->pPI_ID_CURR_CTRL.resetPI=pPFC_CTRL_loc->CDC_Reset;     ///Release PI accumulator
    CDC->pPI_IQ_CURR_CTRL.resetPI=pPFC_CTRL_loc->CDC_Reset;     ///Release PI accumulator

    CDC->Id_feed=Current_qdo->axd;                              ///Pass dq current feedback to CDC struct (d-axis)
    CDC->Iq_feed=Current_qdo->axq;                              ///Pass dq current feedback to CDC struct (q-axis)
    CDC->Vd_feed=Voltage_qdo->axd;                              ///Pass dq voltage feedback to CDC struct (d-axis)
    CDC->Vq_feed=Voltage_qdo->axq;                              ///Pass dq voltage feedback to CDC struct (q-axis)
    CDC->Vdc_feed=VOLTAGE_ADC_DC_IN_PHY->Vdc_tot;               ///Pass DC voltage feedback to CDC struct

    Current_Decoupling_Control(CDC,&CDC->pPI_ID_CURR_CTRL,&CDC->pPI_IQ_CURR_CTRL, &Vd_ctrl_FF,&Vq_ctrl_FF);

    V_DQO_CTRL->axd=Vd_ctrl_FF;
    V_DQO_CTRL->axq=Vq_ctrl_FF;
    V_DQO_CTRL->axo=0;
  }
  else if(pPFC_CTRL_loc->PFC_CTRL_State==OPEN_LOOP) /// Open LOOP
  {
    PI_VDC_CTRL->resetPI=SET;                                   ///Mantein Not used regulator in reset mode
    CDC->pPI_ID_CURR_CTRL.resetPI=SET;                          ///Mantein Not used regulator in reset mode
    CDC->pPI_IQ_CURR_CTRL.resetPI=SET;                          ///Mantein Not used regulator in reset mode

    CDC->Vd_Curr_Ctrl=pPFC_CTRL_loc->V_DQO_CTRL_MAN.axd;
    CDC->Vq_Curr_Ctrl=pPFC_CTRL_loc->V_DQO_CTRL_MAN.axq;

    if(CDC->FF_Enable==SET)
    {
    FeedForward_Control(CDC,&Vd_ctrl_FF,&Vq_ctrl_FF);                                ///dq-axis Feed_Forward applied
    }
    else
    {
    Vd_ctrl_FF=CDC->Vd_Curr_Ctrl;                                                    ///d-axis Feed_Forward bypassed
    Vq_ctrl_FF=CDC->Vq_Curr_Ctrl;                                                    ///q-axis Feed_Forward bypassed
    }

    V_DQO_CTRL->axd=Vd_ctrl_FF;
    V_DQO_CTRL->axq=Vq_ctrl_FF;
    V_DQO_CTRL->axo=0;
  }
  else /// Generate a ERROR!!!
  {
    PI_VDC_CTRL->resetPI=SET;                                   ///Mantein Not used regulator in reset mode
    CDC->pPI_ID_CURR_CTRL.resetPI=SET;                          ///Mantein Not used regulator in reset mode
    CDC->pPI_IQ_CURR_CTRL.resetPI=SET;                          ///Mantein Not used regulator in reset mode

    V_DQO_CTRL->axd=0;
    V_DQO_CTRL->axq=0;
    V_DQO_CTRL->axo=0;
  }

}



/**
* @brief  DPC_LPCNTRL_BURST_Init:
* @param  TBD
*
* @retval none
*
* @note Function valid for STM32G4xx microconroller family
*/
void DPC_LPCNTRL_BURST_Init(BURST_STRUCT *BURST_t_local,FlagStatus Burst_Enable_loc,uint16_t Vref_hist_VOLT,uint16_t delta_Vref_hist_VOLT,float I_dc_NO_LOAD_Limit_AMP_loc,float I_dc_LOW_LOAD_Limit_AMP_loc,float duty_no_load_local,float duty_low_load_local,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc){


  uint16_t Vref_hist_loc;
  uint16_t delta_Vref_hist_loc;
  uint16_t Vout_load_max;                                                                       /*!< Local histeresis higher ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Vout_load_min;                                                                       /*!< Local histeresis lower ouput DC voltage Thrueshold expressed in Bits */
  uint16_t I_dc_NO_LOAD_Limit_loc;                                                              /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate No Load Condition
  uint16_t I_dc_LOW_LOAD_Limit_loc;                                                             /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate Low Load Condition


  Vref_hist_loc=(uint16_t)(((float)Vref_hist_VOLT*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);
  delta_Vref_hist_loc=(uint16_t)(((float)delta_Vref_hist_VOLT*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);
  Vout_load_max=Vref_hist_loc+delta_Vref_hist_loc;                                              /*!< Obtain and set higher output voltage term*/
  Vout_load_min=Vref_hist_loc-delta_Vref_hist_loc;                                              /*!< Obtain and set lower output voltage term*/

  //I_dc_NO_LOAD_Limit_loc=(uint16_t)(((float)I_dc_NO_LOAD_Limit_AMP_loc*DPC_ADC_Conf_loc->G_Idc)+DPC_ADC_Conf_loc->B_Idc);       /// (IDC_No_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  //I_dc_LOW_LOAD_Limit_loc=(uint16_t)(((float)I_dc_LOW_LOAD_Limit_AMP_loc*DPC_ADC_Conf_loc->G_Idc)+DPC_ADC_Conf_loc->B_Idc);     /// (IDC_Low_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_NO_LOAD_Limit_loc = (uint16_t)(float)I_dc_NO_LOAD_Limit_AMP_loc;
  I_dc_LOW_LOAD_Limit_loc=(uint16_t)(float)I_dc_LOW_LOAD_Limit_AMP_loc;


  BURST_t_local->Vref_hist=Vref_hist_loc;
  BURST_t_local->delta_Vref_hist=delta_Vref_hist_loc;
  BURST_t_local->Vout_max=Vout_load_max;
  BURST_t_local->Vout_min=Vout_load_min;
  BURST_t_local->Duty_Limit=0.5;
  BURST_t_local->Duty_noload=duty_no_load_local;
  BURST_t_local->Duty_lowload=duty_low_load_local;
  BURST_t_local->Burst_Enable=Burst_Enable_loc;
  BURST_t_local->Iout_no_load_threshold=I_dc_NO_LOAD_Limit_loc;
  BURST_t_local->Iout_low_load_threshold=I_dc_LOW_LOAD_Limit_loc;

}



/**
* @brief  DPC_LPCNTRL_Burst_Check: Function used to obtain FSM operation during the BURST
* @param  TBD
*
* @retval none
*
* @note Function valid for STM32G4xx microconroller family
*/
BURST_StatusTypeDef DPC_LPCNTRL_Burst_Check(uint32_t* p_Data_Sub, CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub/*uint32_t* iDC_Data_Sub*/,BURST_STRUCT *BURST_CTRL_f){



  BURST_StatusTypeDef BURST_Status;
  uint16_t Vout_load_max;                                                               /*!< Local histeresis higher ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Vout_load_min;                                                               /*!< Local histeresis lower ouput DC voltage Thrueshold expressed in Bits */
  float I_max;
  float I_min;
  int16_t I_max_int;
  int16_t I_min_int;

  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phA){
	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phA;
  }
  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phB){
  	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phB;
    }
  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phC){
  	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phC;
  }

  if (I_min > CURRENT_ADC_AC_IN_NORM_Sub->phA){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phA;
  }
  if (I_min < CURRENT_ADC_AC_IN_NORM_Sub->phB){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phB;
  }
  if (I_min < CURRENT_ADC_AC_IN_NORM_Sub->phC){
	I_min = CURRENT_ADC_AC_IN_NORM_Sub->phC;
  }

  I_min_int = (int16_t) I_min;
  I_max_int = (int16_t) I_max;

  if (-I_min_int > I_max_int){
	  I_max_int= -I_min_int;
  }

  if(BURST_CTRL_f->Burst_Enable==SET){                                                  /** If Burst_Enable is SET */

    BURST_CTRL_f->Vout_load=p_Data_Sub[0]+p_Data_Sub[1];                                /*!< Pass voltages data in local terms ([0]=VDC_upper  [1]=VDC_lower) */
    Vout_load_max=BURST_CTRL_f->Vout_max;                                               /*!< Set higher output voltage term*/
    Vout_load_min=BURST_CTRL_f->Vout_min;                                               /*!< Set lower output voltage term*/                                                                /// [0]=Iload

    if(I_max_int<=(BURST_CTRL_f->Iout_no_load_threshold)){                           ///NO_LOAD  Check
      if (BURST_CTRL_f->Vout_load>Vout_load_max)
      {
        BURST_Status=BURST_Complete;
      }
      else if (BURST_CTRL_f->Vout_load<Vout_load_min)
      {
        BURST_Status=BURST_Progress;
      }
      else
      {
        BURST_Status=BURST_Complete;
      }
    }
    else
    {
      BURST_Status=BURST_Error;
    }
  }
  else{                                                                                 /** If Burst_Enable is RESET */
    BURST_Status=BURST_Disable;
  }

  BURST_CTRL_f->BURST_Status=BURST_Status;
  BURST_CTRL_f->uI_load_Burst=I_max_int;

  return BURST_Status;
}




/**
* @brief  DPC_LPCNTRL_Burst_Mode: Function used for the BURST mode operation
* @param  TBD
*
* @retval none
*
* @note Working with DPC_LPCNTRL_Burst_Check
* @note Function valid for STM32G4xx microconroller family
*/
void DPC_LPCNTRL_Burst_Mode(uint32_t* p_Data_Sub,BURST_STRUCT *BURST_CTRL_f,CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub/*uint32_t* iDC_Data_Sub*/,DPC_PWM_TypeDef *tDPC_PWM_loc,DMA_PWMDUTY_STRUCT* DMA_SOURCE ){

  uint16_t Vout_load;                                                                   /*!< Local actual DPC output voltage expressed in Bits*/
  uint16_t Vout_load_max;                                                               /*!< Local histeresis higher ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Vout_load_min;                                                               /*!< Local histeresis lower ouput DC voltage Thrueshold expressed in Bits */
  float I_max;
  float I_min;
  int16_t I_max_int;
  int16_t I_min_int;
  float Burst_Duty;
  DMA_PWMDUTY_STRUCT* DMA_SOURCE1;

  I_max=0;

  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phA){
	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phA;
  }
  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phB){
  	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phB;
    }
  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phC){
  	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phC;
  }

  if (I_min > CURRENT_ADC_AC_IN_NORM_Sub->phA){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phA;
  }
  if (I_min < CURRENT_ADC_AC_IN_NORM_Sub->phB){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phB;
  }
  if (I_min < CURRENT_ADC_AC_IN_NORM_Sub->phC){
	I_min = CURRENT_ADC_AC_IN_NORM_Sub->phC;
  }

  I_min_int = (int16_t) (I_min*10);
  I_max_int = (int16_t) (I_max*10);

  if (-I_min_int > I_max_int){
	  I_max_int= -I_min_int;
  }

  if (BURST_CTRL_f->Burst_Enable==SET){                                                 /** If Burst_Enable is SET */

    Vout_load=p_Data_Sub[0]+p_Data_Sub[1];                                              /*!< Pass voltages data in local terms ([0]=VDC_upper  [1]=VDC_lower) */
    Vout_load_max=BURST_CTRL_f->Vout_max;                                               /*!< Set higher output voltage term*/
    Vout_load_min=BURST_CTRL_f->Vout_min;                                               /*!< Set lower output voltage term*/
    BURST_CTRL_f->Vout_load=Vout_load;                                                  /*!< Store output voltage in "BURST_CTRL" struct */
    //I_load_Burst=iDC_Data_Sub[0];                                                       /// [0]=Iload


  if(BURST_CTRL_f->BURST_Status==BURST_Progress || BURST_CTRL_f->BURST_Status==BURST_Run){

    if(BURST_CTRL_f->Duty_noload>BURST_CTRL_f->Duty_Limit){                              ///Start Check Duty LIMIT
      BURST_CTRL_f->Duty_noload=BURST_CTRL_f->Duty_Limit;
    }//End Check Duty_noload LIMIT
    if(BURST_CTRL_f->Duty_lowload>BURST_CTRL_f->Duty_Limit){                             ///Start Check Duty LIMIT
      BURST_CTRL_f->Duty_lowload=BURST_CTRL_f->Duty_Limit;
    }//End Check Duty_lowload LIMIT


    if(I_max_int<=(BURST_CTRL_f->Iout_no_load_threshold)){                           ///NO_LOAD  Check
    Burst_Duty=BURST_CTRL_f->Duty_noload;
    }
    else if(I_max_int>(BURST_CTRL_f->Iout_no_load_threshold) || I_max_int<=(BURST_CTRL_f->Iout_low_load_threshold)){                     ///LOW_LOAD  Check
    Burst_Duty=BURST_CTRL_f->Duty_lowload;
    }

    DPC_PWM_Send_Burst_PWM(tDPC_PWM_loc,Burst_Duty,Burst_Duty,Burst_Duty,DMA_SOURCE);  /*!< Refresh BURST Duty*/
    //DMA_SOURCE=DMA_SOURCE1;

//      if (Vout_load>Vout_load_max && BURST_CTRL_f->BURST_PACKAGE==SET)                  /*!< Occured when Vout overcome higher trueshold and BURST_Flag is active*/
      if (Vout_load>Vout_load_max)                                                      /*!< Occured when Vout overcome higher trueshold and BURST_Flag is active*/

      {
        DPC_PWM_OutDisable();                                                           /*!< DISABLE BURST PWM*/
        BURST_CTRL_f->BURST_PACKAGE=RESET;                                              /*!< BURST_Flag become RESET (Burst Inactive)*/
        BURST_CTRL_f->BURST_IN_RANGE=RESET;                                             /*!< BURST_IN_RANGE_Flag become RESET (Vout higher then limit)*/
      }
//      else if (Vout_load<Vout_load_min && BURST_CTRL_f->BURST_PACKAGE==RESET)           /*!< Occured when Vout is lower then low-trueshold and BURST_Flag is stopped*/
      else if (Vout_load<Vout_load_min)                                                 /*!< Occured when Vout is lower then low-trueshold and BURST_Flag is stopped*/
      {
        DPC_PWM_OutEnable(tDPC_PWM_loc);                                               /*!< ENABLE BURST PWM*/
        BURST_CTRL_f->BURST_PACKAGE=SET;                                                /*!< BURST_Flag become SET (Burst Active)*/
        BURST_CTRL_f->BURST_IN_RANGE=RESET;                                             /*!< BURST_IN_RANGE_Flag become RESET (Vout lower then limit)*/
      }
      else                                                                              /*!< Occured in inner hysteresis window*/
      {
        BURST_CTRL_f->BURST_IN_RANGE=SET;
      }
  }
  }
  BURST_CTRL_f->uI_load_Burst=I_max;
  BURST_CTRL_f->Burst_Duty=Burst_Duty;
}





/**
  * @brief  DPC_LPCNTRL_Inrush_Check: Function used to obtain FSM operation during the INRUSH
  * @param  TBD
  *
  * @retval INRUSH_StatusTypeDef
  *
  * @note Function valid for STM32G4xx microconroller family
  */
INRUSH_StatusTypeDef DPC_LPCNTRL_Inrush_Check(uint32_t* p_Data_Sub,CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub/* uint32_t* iDC_Data_Sub*/,INRUSH_STRUCT *INRUSH_CTRL_f){

  INRUSH_StatusTypeDef  INRUSH_Status;                                                  /*!< Local INRUSH_StatusTypeDef */
  uint16_t Vout_load;                                                                   /*!< Local actual DPC output voltage expressed in Bits*/
  uint16_t I_load_Inrush;                                                               /*!< Local actual DPC output current expressed in Bits*/
  uint16_t Vout_load_max;                                                               /*!< Local histeresis higher ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Vout_load_min;                                                               /*!< Local histeresis lower ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Iout_load_threshold;
  float I_max;
  float I_min;
  int16_t I_max_int;
  int16_t I_min_int;

  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phA){
	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phA;
  }
  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phB){
  	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phB;
    }
  if (I_max < CURRENT_ADC_AC_IN_NORM_Sub->phC){
  	  I_max = CURRENT_ADC_AC_IN_NORM_Sub->phC;
  }

  if (I_min > CURRENT_ADC_AC_IN_NORM_Sub->phA){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phA;
  }
  if (I_min < CURRENT_ADC_AC_IN_NORM_Sub->phB){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phB;
  }
  if (I_min < CURRENT_ADC_AC_IN_NORM_Sub->phC){
	  I_min = CURRENT_ADC_AC_IN_NORM_Sub->phC;
  }

  I_min_int = (int16_t) (I_min*10);
  I_max_int = (int16_t) (I_max*10);

  if (-I_min_int > I_max_int){
	  I_max_int= -I_min_int;
  }

  if (INRUSH_CTRL_f->InrushEnable==SET){                                                /** If InrushEnable is SET */

    Vout_load=p_Data_Sub[0]+p_Data_Sub[1];                                              /*!< Pass voltages data in local terms ([0]=VDC_upper  [1]=VDC_lower) */
    //I_load_Inrush=iDC_Data_Sub[0];                                                      /*!< Pass current data in local terms [0]=Iload */
    I_load_Inrush=(uint16_t) I_max_int;
    INRUSH_CTRL_f->Vout_load=Vout_load;                                                 /*!< Store output voltage in "INRUSH" struct */
    Vout_load_max=INRUSH_CTRL_f->Vout_max;                                              /*!< Set higher output voltage term*/
    Vout_load_min=INRUSH_CTRL_f->Vout_min;                                              /*!< Set lower output voltage term*/
    Iout_load_threshold=INRUSH_CTRL_f->Iout_load_threshold;                             /*!< Set output current term*/

    if(I_load_Inrush<=Iout_load_threshold){                                             /*!< NO_LOAD  Check */
      if (Vout_load>Vout_load_max)                                                      /*!< ERROR Check - If occur AC OVERVOLTAGE or 3W-4W are not properly configurated*/
      {
        INRUSH_Status=INRUSH_Error;
      }
      else if (Vout_load<Vout_load_min)                                                 /*!< Inrush Check - If NOT occured AC UNDERVOLTAGE or 3W-4W are not properly configurated*/
      {
        INRUSH_Status=INRUSH_Progress;
      }
      else                                                                              /** InrushEnable is SET and completed*/
      {
        INRUSH_Status=INRUSH_Complete;
      }
    }///  END NO_LOAD  Check
    else                                                                                /** InrushEnable is SET but DC current is present during the inrush (ERROR) */
    {
      INRUSH_Status=INRUSH_Error;
    }
  }
  else                                                                                  /** If InrushEnable is RESET */
  {
    INRUSH_Status=INRUSH_Disable;
  }

  INRUSH_CTRL_f->INRUSH_Status=INRUSH_Status;
  INRUSH_CTRL_f->I_load_Inrush=I_load_Inrush;

  return INRUSH_Status;
}





/**
  * @brief  DPC_LPCNTRL_Inrush_Init:
  * @param  TBD
  *
  * @retval INRUSH_StatusTypeDef
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void DPC_LPCNTRL_Inrush_Init(INRUSH_STRUCT *INRUSH_CTRL_f,uint16_t Vref_hist_VOLT_loc,uint16_t delta_Vref_hist_VOLT_loc,float I_dc_NO_LOAD_Limit_AMP_loc,FlagStatus InrushEnable_loc,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc)
{


  uint16_t Vref_hist_loc;
  uint16_t delta_Vref_hist_loc;
  uint16_t Vout_load_max;                                                                       /*!< Local histeresis higher ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Vout_load_min;                                                                       /*!< Local histeresis lower ouput DC voltage Thrueshold expressed in Bits */
  uint16_t I_dc_NO_LOAD_Limit_loc;                                                              /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate No Load Condition



  Vref_hist_loc=(uint16_t)(((float)Vref_hist_VOLT_loc*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);
  delta_Vref_hist_loc=(uint16_t)(((float)delta_Vref_hist_VOLT_loc*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);

  Vout_load_max=Vref_hist_loc+delta_Vref_hist_loc;                                              /*!< Obtain and set higher output voltage term*/
  Vout_load_min=Vref_hist_loc-delta_Vref_hist_loc;                                              /*!< Obtain and set lower output voltage term*/

  //I_dc_NO_LOAD_Limit_loc=(uint16_t)(((float)I_dc_NO_LOAD_Limit_AMP_loc*DPC_ADC_Conf_loc->G_Idc)+DPC_ADC_Conf_loc->B_Idc);   /// (IDC_No_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_NO_LOAD_Limit_loc=(uint16_t)I_dc_NO_LOAD_Limit_AMP_loc;



  INRUSH_CTRL_f->Vref_hist=Vref_hist_loc;
  INRUSH_CTRL_f->delta_Vref_hist=delta_Vref_hist_loc;
  INRUSH_CTRL_f->InrushEnable=InrushEnable_loc;
  INRUSH_CTRL_f->Vout_max=Vout_load_max;
  INRUSH_CTRL_f->Vout_min=Vout_load_min;
  INRUSH_CTRL_f->Iout_load_threshold=I_dc_NO_LOAD_Limit_loc;
}






/**
  * @brief  DPC_LPCNTRL_CDC_Init:
  * @param  TBD
  *
  * @retval none
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void DPC_LPCNTRL_CDC_Init(CDC_Struct *CDC_local,float omegagrid_loc,float Inductor_loc,FlagStatus FF_Enable_SET,FlagStatus Decoupling_Enable_SET,FlagStatus VDC_FF_Enable_SET)
{
  CDC_local->omegagrid=omegagrid_loc;                   /*!< Set omega in Current Decaupling Control*/
  CDC_local->Inductor=Inductor_loc;                     /*!< Set Inductor value in Current Decaupling Control*/
  CDC_local->FF_Enable=FF_Enable_SET;                   /*!< Set Initial state of AC FeedForward in Current Decoupling Control*/
  CDC_local->Decoupling_Enable=Decoupling_Enable_SET;   /*!< Set Initial state of Decoupling compensator in Current Decoupling Control*/
  CDC_local->VDC_FF_Enable=VDC_FF_Enable_SET;           /*!< Set Initial state of DC FeedForward in Current Decoupling Control*/
}



///**
//  * @brief  DPC_CTRL_RELAY_ChangeState:
//  * @param  TBD
//  * 
//  * @retval TBD 
//  *
//  * @note Function valid for STM32G4xx microconroller family  
//  */ 
//void DPC_CTRL_RELAY_ChangeState(Relay_Typedef *Relay_local)
//{
//
//  if(Relay_local->RELAY_INRSH_State==SET){HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);}
//  if(Relay_local->RELAY_GRID_A_State==SET){HAL_GPIO_WritePin(ZVD_C__RLY_A_GPIO_Port, ZVD_C__RLY_A_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(ZVD_C__RLY_A_GPIO_Port, ZVD_C__RLY_A_Pin, GPIO_PIN_RESET);}
//  if(Relay_local->RELAY_GRID_B_State==SET){HAL_GPIO_WritePin(ZVD_B__RLY_B_GPIO_Port, ZVD_B__RLY_B_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(ZVD_B__RLY_B_GPIO_Port, ZVD_B__RLY_B_Pin, GPIO_PIN_RESET);}
//  if(Relay_local->RELAY_GRID_C_State==SET){HAL_GPIO_WritePin(ZVD_A__RLY_C_GPIO_Port, ZVD_A__RLY_C_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(ZVD_A__RLY_C_GPIO_Port, ZVD_A__RLY_C_Pin, GPIO_PIN_RESET);}
//  if(Relay_local->FAN_State==SET){HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);}
//}



/**
  * @brief  DPC_Relay_Init:
  * @param  TBD
  *
  * @retval TBD
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void DPC_Relay_Init(Relay_Typedef *Relay_local)
{
Relay_local->RELAY_INRSH_State=SET;
Relay_local->RELAY_GRID_A_State=SET;
Relay_local->RELAY_GRID_B_State=SET;
Relay_local->RELAY_GRID_C_State=SET;
Relay_local->FAN_State=SET;
}



/**
* @brief  DPC_LPCNTRL_PFC_Init:
* @param  TBD
*
* @retval TBD
*
* @note Function valid for STM32G4xx microconroller family
*/
//void DPC_LPCNTRL_PFC_Init(PFC_CTRL_t *pPFC_CTRL,PFC_CTRL_State_TypeDef PFC_CTRL_State,uint16_t PFC_VDC_Ref_loc,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc);
void DPC_LPCNTRL_PFC_Init(PFC_CTRL_t *pPFC_CTRL_loc,PFC_CTRL_State_TypeDef PFC_CTRL_State,uint16_t PFC_VDC_Ref_loc,DPC_ADC_Conf_TypeDef *DPC_ADC_Conf_loc)
{
  uint16_t PFC_VDC_Ref_BITs_loc;                                                                                /// Local variable to pass Output voltage reference  (Expressed in BITs)
  PFC_VDC_Ref_BITs_loc=(uint16_t)(((float)PFC_VDC_Ref_loc*DPC_ADC_Conf_loc->G_Vdc)+DPC_ADC_Conf_loc->B_Vdc);   /// (V_dc_ref [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias

  pPFC_CTRL_loc->PFC_VDC_Ref=PFC_VDC_Ref_loc;
  pPFC_CTRL_loc->PFC_VDC_Ref_BITs=PFC_VDC_Ref_BITs_loc;
  pPFC_CTRL_loc->CDC_Reset=RESET;
  pPFC_CTRL_loc->VdcCTRL_Reset=RESET;
  pPFC_CTRL_loc->PFC_CTRL_State=PFC_CTRL_State;
}

/**
* @brief  DPC_LPCNTRL_Burst_PID_Mode:
* @param  TBD
*
* @retval TBD
*
* @note Function valid for STM32G4xx microconroller family
*/
void DPC_LPCNTRL_Burst_PID_Mode(uint32_t* p_Data_Sub,BURST_STRUCT *BURST_CTRL_f,CurrentAC_ADC_NORM_Struct* CURRENT_ADC_AC_IN_NORM_Sub/*uint32_t* iDC_Data_Sub*/,DPC_PWM_TypeDef *tDPC_PWM_loc,DMA_PWMDUTY_STRUCT* DMA_SOURCE ){

  uint16_t Vout_load;                                                                   /*!< Local actual DPC output voltage expressed in Bits*/
  uint16_t Vout_load_max;                                                               /*!< Local histeresis higher ouput DC voltage Thrueshold expressed in Bits */
  uint16_t Vout_load_min;                                                               /*!< Local histeresis lower ouput DC voltage Thrueshold expressed in Bits */
  float I_max;
  float I_min;
  int16_t I_max_int;
  int16_t I_min_int;
  float Burst_Duty;
  DMA_PWMDUTY_STRUCT* DMA_SOURCE1;

  I_max=0;



  if (BURST_CTRL_f->Burst_Enable==SET){                                                 /** If Burst_Enable is SET */
    Vout_load=p_Data_Sub[0]+p_Data_Sub[1];                                              /*!< Pass voltages data in local terms ([0]=VDC_upper  [1]=VDC_lower) */
    Vout_load_max=BURST_CTRL_f->Vout_max;                                               /*!< Set higher output voltage term*/
    Vout_load_min=BURST_CTRL_f->Vout_min;                                               /*!< Set lower output voltage term*/
    BURST_CTRL_f->Vout_load=Vout_load;                                                  /*!< Store output voltage in "BURST_CTRL" struct */


  if(BURST_CTRL_f->BURST_Status==BURST_Progress || BURST_CTRL_f->BURST_Status==BURST_Run){

    if(BURST_CTRL_f->Duty_noload>BURST_CTRL_f->Duty_Limit){                              ///Start Check Duty LIMIT
      BURST_CTRL_f->Duty_noload=BURST_CTRL_f->Duty_Limit;
    }//End Check Duty_noload LIMIT
    if(BURST_CTRL_f->Duty_lowload>BURST_CTRL_f->Duty_Limit){                             ///Start Check Duty LIMIT
      BURST_CTRL_f->Duty_lowload=BURST_CTRL_f->Duty_Limit;
    }//End Check Duty_lowload LIMIT


    if(I_max_int<=(BURST_CTRL_f->Iout_no_load_threshold)){                           ///NO_LOAD  Check
    Burst_Duty=BURST_CTRL_f->Duty_noload;
    }
    else if(I_max_int>(BURST_CTRL_f->Iout_no_load_threshold) || I_max_int<=(BURST_CTRL_f->Iout_low_load_threshold)){                     ///LOW_LOAD  Check
    Burst_Duty=BURST_CTRL_f->Duty_lowload;
    }

    DPC_PWM_Send_Burst_PWM(tDPC_PWM_loc,Burst_Duty,Burst_Duty,Burst_Duty,DMA_SOURCE);  /*!< Refresh BURST Duty*/
    //DMA_SOURCE=DMA_SOURCE1;

//      if (Vout_load>Vout_load_max && BURST_CTRL_f->BURST_PACKAGE==SET)                  /*!< Occured when Vout overcome higher trueshold and BURST_Flag is active*/
      if (Vout_load>Vout_load_max)                                                      /*!< Occured when Vout overcome higher trueshold and BURST_Flag is active*
      {
        DPC_PWM_OutDisable();                                                           /*!< DISABLE BURST PWM*/
        BURST_CTRL_f->BURST_PACKAGE=RESET;                                              /*!< BURST_Flag become RESET (Burst Inactive)*/
        BURST_CTRL_f->BURST_IN_RANGE=RESET;                                             /*!< BURST_IN_RANGE_Flag become RESET (Vout higher then limit)*/
      }
//      else if (Vout_load<Vout_load_min && BURST_CTRL_f->BURST_PACKAGE==RESET)           /*!< Occured when Vout is lower then low-trueshold and BURST_Flag is stopped*/
      else if (Vout_load<Vout_load_min)                                                 /*!< Occured when Vout is lower then low-trueshold and BURST_Flag is stopped*/
      {
        DPC_PWM_OutEnable(tDPC_PWM_loc);                                               /*!< ENABLE BURST PWM*/
        BURST_CTRL_f->BURST_PACKAGE=SET;                                                /*!< BURST_Flag become SET (Burst Active)*/
        BURST_CTRL_f->BURST_IN_RANGE=RESET;                                             /*!< BURST_IN_RANGE_Flag become RESET (Vout lower then limit)*/
      }
      else                                                                              /*!< Occured in inner hysteresis window*/
      {
        BURST_CTRL_f->BURST_IN_RANGE=SET;
      }
  }
  }
  BURST_CTRL_f->uI_load_Burst=I_max;
  BURST_CTRL_f->Burst_Duty=Burst_Duty;
}

