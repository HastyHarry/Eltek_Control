/**
  ******************************************************************************
  * @file    Transform.h
  * @brief   This file contains the headers of the transform module.
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
#ifndef __TRANSFORMS_H
#define __TRANSFORMS_H

/* Includes ------------------------------------------------------------------*/
#include "DPC_Datacollector.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Park(float alpha, float beta, float theta,float phi, float *d, float *q);
void InvPark(float d, float q,float theta,float phi, float *alpha, float *beta);
void Clarke(float a, float b,float c,float *alpha, float *beta);
void Clarke_NORM(float a, float b, float *alpha, float *beta);
void InvClarke(float alpha, float beta, float *a, float *b, float *c);
void InvClarke_opt(float alpha, float beta, float *a, float *b, float *c);
void Clarke_Park(float a, float b, float c, float theta,float phi, float *d, float *q,float *o);
void Clarke_Park_Opt(float a, float b, float c, float theta, float *d, float *q,float *o);
void inv_Clarke_Park(float d, float q, float o, float theta, float phi,float *a, float *b,float *c);
void inv_Clarke_Park_opt(float d, float q, float o, float theta,float phi, float *a, float *b,float *c);
void Run_ClarkePark(TRANSFORM_ABC_t* abc_sub,float theta_sub,float phi_sub,TRANSFORM_QDO_t* out_cl_pa_sub );
void Run_ClarkePark_Opt(TRANSFORM_ABC_t* abc_sub,float theta_sub,TRANSFORM_QDO_t* out_cl_pa_sub );
void Run_Inv_ClarkePark(TRANSFORM_QDO_t* dqo_sub,float theta_sub,float phi_sub,TRANSFORM_ABC_t* out_inv_cl_pa_sub );
void Run_Inv_ClarkePark_opt(TRANSFORM_QDO_t* dqo_sub,float theta_sub,float phi_sub,TRANSFORM_ABC_t* out_inv_cl_pa_sub );

    
#endif /* __TRANSFORMS_H*/
    