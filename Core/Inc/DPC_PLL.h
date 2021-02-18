/**
  ******************************************************************************
  * @file    PLL.h
  * @brief   This file contains the headers of the Database.
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
#ifndef __PLL_H
#define __PLL_H

/* Includes ------------------------------------------------------------------*/

#include "DPC_Datacollector.h"
#include "DPC_Math.h"


/* Exported types ------------------------------------------------------------*/

/** 
 *@brief
 */
typedef struct{
  float VphA;                                   /*!< */
  float VphB;                                   /*!< */
  float VphC;                                   /*!< */
}VoltageAC_PLL_Struct;

/** 
 *@brief 
 */
typedef struct{
  float Vph_d;                                  /*!< */
  float Vph_q;                                  /*!< */
  float Vph_o;                                  /*!< */
}VoltageAC_qd_PLL_Struct;


/** 
 *@brief 
 */
typedef struct{
  uint32_t var_d;                               /*!< */
  uint32_t var_q;                               /*!< */
  uint32_t var_o;                               /*!< */
}PLL_qdo_Struct;

/** 
 *@brief PLL:
 */
typedef enum
{
PLL_IDLE = 0,                                   /*!< */
PLL_SYNC,                                       /*!< */
PLL_OUTRANGE,                                   /*!< */
PLL_FAULT,                                      /*!< */
PLL_ERROR,                                      /*!< */
PLL_DISABLED,                                   /*!< */
}
STATUS_PLL_TypeDef;                             /*!< */

/** 
 *@brief PLL: Struct Variables
 */
typedef struct{
  float pll_theta_in;                           /*!< */
  float pll_theta_out;                          /*!< */ 
  float pll_theta_out_2pi;                      /*!< */
  float pll_phi_2pi;                            /*!< */
  float pll_d;                                  /*!< */
  float pll_q;                                  /*!< */
  float pll_o;                                  /*!< */
  float kp_pll;                                 /*!< */
  float ki_pll;                                 /*!< */
  float k0_pll;                                 /*!< */
  float k1_pll;                                 /*!< */
  float Ts_pll;                                 /*!< */
  float omega_piout;                            /*!< */
  float omega_ff_pll;                           /*!< */
  PI_STRUCT_t pi_pll;                           /*!< */
  INTEGRATOR_STRUCT integrator_pll;             /*!< */
  float delta_freq;                             /*!< */
  STATUS_PLL_TypeDef Status_PLL;                /*!< */
  FlagStatus PLL_Enable;                        /*!< */
  uint8_t uFreqFeedforwardHz;                   /*!< FeedForward term of PLL (expressed in Hz i.e. 50Hz or 60Hz)   */
}PLL_Struct;
  

  
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DPC_PLL_Init(PLL_Struct *PLL_sub, float kp_pll, float ki_pll, float Ts_pll,float phi_2pi,float delta_freq, uint8_t uFeedforward_Hz, FlagStatus satPI_toggle,float PIsat_up,float PIsat_down);
void PLLabc(PLL_Struct *PLL_sub, VoltageAC_PLL_Struct *VAC_PLL,float *theta_out,float *omega_pi_out);
void PLLabc_opt(PLL_Struct *PLL_sub, VoltageAC_PLL_Struct *VAC_PLL,float *theta_out,float *omega_pi_out);
STATUS_PLL_TypeDef DPC_PLL_pllqd_Run(PLL_Struct *PLL_sub, VoltageAC_qd_PLL_Struct *VAC_qd_PLL,float *theta_out,  float *omega_pi_out);



#endif /*__PLL_H */ 