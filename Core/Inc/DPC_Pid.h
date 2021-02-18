/**
  ******************************************************************************
  * @file    Pid.h
  * @brief   This file contains the headers of the Pid Module.
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
#ifndef __PID_H
#define __PID_H


/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct {
  float Ref;
  float Feed;
  float Kp; 
  float Ki;
  float Ts;
  float Integral;
  float PIout;
  float PIout_sat;
  float PIsat_up;
  float PIsat_down;
  float error;
  float Integralout;
  FlagStatus resetPI;  
  float k0;
  float k1;
  float Antiwindup_Term;
  FlagStatus satPI_toggle;
  FlagStatus antiwindPI_toggle;
  float Antiwindup_Gain;
}PI_STRUCT_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DPC_PI_Init(PI_STRUCT_t *pPI,float Init_Val_Kp,float Init_Val_Ki,float Init_Val_Ts,float Init_PIsat_up, float Init_PIsat_down,FlagStatus satPI_toggle_local,FlagStatus antiwindPI_toggle_local,float Antiwindup_Gain_local);
float PI(float Ref, float Feed , PI_STRUCT_t *pPI);
void PI_RESET(PI_STRUCT_t *pPI);

 
#endif