/**
  ******************************************************************************
  * @file    DPC_Faulterror.h
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
#ifndef __DPC_FAULTERROR_H
#define __DPC_FAULTERROR_H


/* Includes ------------------------------------------------------------------*/
#include "DPC_PWMConverter.h"
/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Type of control activated OPEN - INNER - OUTER
  */
typedef enum
{
  NO_FAULTERROR =0,
  FAULT_TRUE,
  ERROR_TRUE,
  NOT_ERASABLE,
  ERASE_ERROR_LIST_NOT_EMPTY,
  ERASE_OK,
 } DPC_FAULTERROR_STATUS_TypeDef;

typedef enum
{
  NO_FAULT =0,
  FAULT_OCL             = 0x00000001,           //OVER CURRENT LOAD
  FAULT_OVL             = 0x00000002,           //OVER VOLTAGE LOAD
  FAULT_OVC             = 0x00000004,           //OVER VOLTAGE CAPACITOR
  FAULT_OCS             = 0x00000008,           //OVER CURRENT SOURCE
  FAULT_OVS             = 0x00000010,           //OVER VOLTAGE SOURCE
  FAULT_INR             = 0x00000020,           //INRUSH
  FAULT_BRS             = 0x00000040,           //BURST
  FAULT_PLL_OR          = 0x00000080,           //PLL Out of Range  
  FAULT_PFC_UVLO        = 0x00000100,           //FAULT Under-Voltage-AC
  FAULT_IDLE            = 0x00000200,           //FAULT_IDLE
  FAULT_GEN             = 0x00000400,           //TBD
  FAULT_012             = 0x00000800,           //TBD
  FAULT_013             = 0x00001000,           //TBD
  FAULT_014             = 0x00002000,           //TBD
  FAULT_015             = 0x00004000,           //TBD
  FAULT_016             = 0x00008000,           //TBD  
  ERROR_PLL             = 0x00010000,           //ERROR PLL
  ERROR_IDLE            = 0x00020000,           //ERROR IDLE
  ERROR_START_INRS      = 0x00040000,           //ERROR START-UP INRUSH
  ERROR_FSM             = 0x00080000,           //ERROR in Finite State Machine
  ERROR_PFC_UVLO        = 0x00100000,           //ERROR AC UnderVoltage during PFC mode
  ERROR_BRS             = 0x00200000,           //TBD
  ERROR_AC_UV           = 0x00400000,           //ERROR Under-Voltage-AC
  ERROR_PLL_OR          = 0x00800000,           //ERROR PLL Out of Range
  ERROR_PFC_RUN         = 0x01000000,           //ERROR_PFC_RUN
  ERROR_AC_UVLO         = 0x02000000,           //ERROR Under-Voltage-Lockout AC
  ERROR_AC_OFF          = 0x04000000,           //ERROR No AC 
  ERROR_PFC             = 0x08000000,           //ERROR during PFC mode
  ERROR_013             = 0x10000000,           //TBD
  ERROR_014             = 0x20000000,           //TBD  
  ERROR_015             = 0x40000000,           //TBD
  ERROR_016             = 0x80000000,           //TBD    
} DPC_FAULTERROR_LIST_TypeDef;


/* Exported constants --------------------------------------------------------*/
#define FAULT_MASK      0x0000FFFF
#define ERROR_MASK      0xFFFF0000
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
DPC_FAULTERROR_LIST_TypeDef DPC_FLT_Faulterror_Check(void);
void DPC_FLT_Faulterror_Set(DPC_FAULTERROR_LIST_TypeDef eFaulterror);
DPC_FAULTERROR_STATUS_TypeDef DPC_FLT_Error_Reset(DPC_FAULTERROR_LIST_TypeDef eError);


#endif /* __DPC_FAULTERROR_H */