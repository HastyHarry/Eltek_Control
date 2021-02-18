/**
******************************************************************************
* @file           : PID.c
* @brief          : PI Module
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
#include "DPC_Pid.h"

/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  DPC_PI_Init: Init PI Data Struct.
* @param  pPI: pointer to a PI_STRUCT_t  that contains
*         the configuration information and data for the specified PI. 
*
* @param  Init_Val_Kp: Init Kp data
* @param  Init_Val_Ki: Init Ki data
* @param  Init_Val_Ts: Init Ts data
* @param  Init_PIsat_up: Init PI saturation Up data
* @param  Init_PIsat_down: Init PI saturation Down data
* @param  uFeedforward_Hz: FeedForward term of PLL (expressed in Hz i.e. 50Hz or 60Hz)
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/  
void DPC_PI_Init(PI_STRUCT_t *pPI,float Init_Val_Kp,float Init_Val_Ki,float Init_Val_Ts,float Init_PIsat_up, float Init_PIsat_down,FlagStatus satPI_toggle_local,FlagStatus antiwindPI_toggle_local,float Antiwindup_Gain_local)
{
  pPI->Kp=Init_Val_Kp;
  pPI->Ki=Init_Val_Ki;
  pPI->Ts=Init_Val_Ts;
  pPI->Integral=0;
  pPI->PIout=0;
  pPI->PIsat_up=Init_PIsat_up;
  pPI->PIsat_down=Init_PIsat_down;
  pPI->error=0;
  pPI->Integralout=0;
  pPI->resetPI=RESET;
  pPI->k0=Init_Val_Kp; //K0=Kp
  pPI->k1=Init_Val_Ki*Init_Val_Ts; //K1=Ki*Ts
  pPI->satPI_toggle=satPI_toggle_local;
  pPI->antiwindPI_toggle=antiwindPI_toggle_local;
  pPI->Antiwindup_Gain=Antiwindup_Gain_local;
}

/**
* @brief  PI: PI function
* @param  Ref: 
* @param  Feed: 
* @param  pPI: pointer to a PI_STRUCT_t  that contains
*         the configuration information and data for the specified PI. 
*
* @retval float Return output data of PI regulator
*
* @note Function valid for STM32G4xx microconroller family  
*/
float PI(float Ref, float Feed , PI_STRUCT_t *pPI)
{
pPI->Ref=Ref;
pPI->Feed=Feed;

  if(pPI->resetPI==SET)
  {
    pPI->Integral=0;
  }
  else{
    pPI->error=(float)Ref-(float)Feed;
    pPI->Integral=pPI->Integral+(pPI->k1*pPI->error)+pPI->Antiwindup_Term;
    pPI->Integralout=pPI->Integral;
    pPI->PIout=(pPI->k0*pPI->error)+pPI->Integralout;
  }

  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(    pPI->PIout>pPI->PIsat_up)
    {
      pPI->PIout_sat=pPI->PIsat_up;
    }
    else if(    pPI->PIout<pPI->PIsat_down)
    {
      pPI->PIout_sat=pPI->PIsat_down;
    }
    else {
      pPI->PIout_sat=pPI->PIout;
    }
     
    //Start Check Antiwindup
    if (pPI->antiwindPI_toggle==SET){
      //Saturation
      pPI->Antiwindup_Term=(pPI->PIout_sat-pPI->PIout)*pPI->Antiwindup_Gain;
    }
    else {
      pPI->Antiwindup_Term=0;
    }
    //End Check Antiwindup    
  }
  else {
    pPI->PIout_sat=pPI->PIout;  
    pPI->Antiwindup_Term=0;
  }
  //End Check Saturation
  
  return pPI->PIout_sat;  
}



/**
* @brief  Reset PI value
* @param  pPI: pointer to a PI_STRUCT_t  that contains
*         the configuration information and data for the specified PI. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void PI_RESET(PI_STRUCT_t *pPI)
{
  pPI->Integral=0;
}

