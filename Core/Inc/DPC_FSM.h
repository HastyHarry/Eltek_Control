/**
  ******************************************************************************
<<<<<<< HEAD
  * @file    DPC_FSM.h
=======
  * @file    Pid.h
>>>>>>> fb3595b88dbc172cf412553db55fda64bb6ec50d
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
#ifndef __FSM_H
#define __FSM_H


/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

/* Exported types ------------------------------------------------------------*/

typedef enum
{
DPC_FSM_WAIT,       	/*!< Persistent state where the application is intended to stay
                     after the fault conditions disappeared for a defined wait time.
                     Following state is normally IDLE */

DPC_FSM_IDLE,           /*!< Persistent state, following state can be INIT if all required 
                      input conditions are satisfied or STOP if there is a fault 
                      condition */

DPC_FSM_INIT,          /*!<"Pass-through" state, the code to be executed only once 
                     between IDLE and START states to initialize control variables.
                     Following state is normally START but it can
                     also be STOP if there is a fault condition */

DPC_FSM_START,          /*!< Persistent state where the converter's start-up is intended to be
                     executed. The following state is normally 
                     RUN as soon the bus reference voltage is reached. Another 
                     possible following state is STOP if there is a fault condition */

DPC_FSM_RUN,            /*!< Persistent state with converter running. The following state 
                     is STOP if there is a fault condition */

DPC_FSM_STOP,           /*!< "Pass-through" state where PWM are disabled. The state machine 
                      can be moved from any condition directly to this state by 
                      SMPS_FaultCheck function. following state is ERROR/FAULT */

DPC_FSM_ERROR,          /*!< Invalid state or wrong value potential fault. Following state is normally 
                      WAIT as soon error/faults are cleared (the error can be recovered without 
                      a system reset, the fault must be recovered with a system reset)*/

DPC_FSM_FAULT,          /*!< Persistent state after a fault. Following state is are recovered with 
                          a system reset*/


} DPC_FSM_State_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DPC_FSM_Application(void);  
bool DPC_FSM_WAIT_Func(void);
bool DPC_FSM_IDLE_Func(void);
bool DPC_FSM_INIT_Func(void);
bool DPC_FSM_START_Func(void);
bool DPC_FSM_RUN_Func(void);
bool DPC_FSM_STOP_Func(void);
bool DPC_FSM_ERROR_Func(void);
bool DPC_FSM_FAULT_Func(void);
DPC_FSM_State_t DPC_FSM_State_Get(void);
void DPC_FSM_State_Set(DPC_FSM_State_t);

#endif //__FSM_H