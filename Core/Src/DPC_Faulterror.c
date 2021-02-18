/**
  ******************************************************************************
  * @file           : DPC_Faulterror.c
  * @brief          : Fault and Error  modulation management
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
#include "DPC_Faulterror.h"




/* external variables ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint32_t uwFaultErrorVector;

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  DPC_FLT_Faulterror_Check: Check if an error or fault was occours 
  *         
  * @param None  
  *
  * @retval DPC_FAULTERROR_LIST_TypeDef: error or fault occours, hiest priority
  *
  * @note Function valid for STM32G4xx and STM32F74x microconroller family   
  */
DPC_FAULTERROR_LIST_TypeDef DPC_FLT_Faulterror_Check(void){

DPC_FAULTERROR_LIST_TypeDef uwFaErVecLocal=NO_FAULT;

if(uwFaultErrorVector != NO_FAULT){

  if(uwFaultErrorVector & FAULT_OCL){
    uwFaErVecLocal = FAULT_OCL;
  }
  else if(uwFaultErrorVector & FAULT_OVL){
    uwFaErVecLocal = FAULT_OVL;
  }
  else if(uwFaultErrorVector & FAULT_OVC){
    uwFaErVecLocal = FAULT_OVC;
  }
  else if(uwFaultErrorVector & FAULT_OCS){
    uwFaErVecLocal = FAULT_OCS;
  }
  else if(uwFaultErrorVector & FAULT_OVS){
    uwFaErVecLocal = FAULT_OVS;
  }
  else if(uwFaultErrorVector & FAULT_INR){
    uwFaErVecLocal = FAULT_INR;
  }
  else if(uwFaultErrorVector & FAULT_BRS){
    uwFaErVecLocal = FAULT_BRS;
  }
  else if(uwFaultErrorVector & FAULT_PLL_OR){
    uwFaErVecLocal = FAULT_PLL_OR;
  }  
  else if(uwFaultErrorVector & FAULT_PFC_UVLO){
    uwFaErVecLocal = FAULT_PFC_UVLO;
  }  
  else if(uwFaultErrorVector & FAULT_IDLE){
    uwFaErVecLocal = FAULT_IDLE;
  }  
  else if(uwFaultErrorVector & FAULT_GEN){
    uwFaErVecLocal = FAULT_GEN;
  }  
  else if(uwFaultErrorVector & FAULT_012){
    uwFaErVecLocal = FAULT_012;
  }  
  else if(uwFaultErrorVector & FAULT_013){
    uwFaErVecLocal = FAULT_013;
  }  
  else if(uwFaultErrorVector & FAULT_014){
    uwFaErVecLocal = FAULT_014;
  }    
  else if(uwFaultErrorVector & FAULT_015){
    uwFaErVecLocal = FAULT_015;
  }    
  else if(uwFaultErrorVector & FAULT_016){
    uwFaErVecLocal = FAULT_016;
  }      
  // ADD new Fault Index
  else if(uwFaultErrorVector & ERROR_PLL){
    uwFaErVecLocal = ERROR_PLL;
  }  
  else if(uwFaultErrorVector & ERROR_IDLE){
    uwFaErVecLocal = ERROR_IDLE;
  }  
  else if(uwFaultErrorVector & ERROR_START_INRS){
    uwFaErVecLocal = ERROR_START_INRS;
  }
  else if(uwFaultErrorVector & ERROR_FSM){
    uwFaErVecLocal = ERROR_FSM;
  }
  else if(uwFaultErrorVector & ERROR_PFC_UVLO){
    uwFaErVecLocal = ERROR_PFC_UVLO;
  }
  else if(uwFaultErrorVector & ERROR_BRS){
    uwFaErVecLocal = ERROR_BRS;
  }
  else if(uwFaultErrorVector & ERROR_AC_UV){
    uwFaErVecLocal = ERROR_AC_UV;
  }
  else if(uwFaultErrorVector & ERROR_PLL_OR){
    uwFaErVecLocal = ERROR_PLL_OR;
  }
  else if(uwFaultErrorVector & ERROR_PFC_RUN){
    uwFaErVecLocal = ERROR_PFC_RUN;
  }
  else if(uwFaultErrorVector & ERROR_AC_UVLO){
    uwFaErVecLocal = ERROR_AC_UVLO;
  }
  else if(uwFaultErrorVector & ERROR_AC_OFF){
    uwFaErVecLocal = ERROR_AC_OFF;
  }
  else if(uwFaultErrorVector & ERROR_PFC){
    uwFaErVecLocal = ERROR_PFC;
  }
  else if(uwFaultErrorVector & ERROR_013){
    uwFaErVecLocal = ERROR_013;
  }
  else if(uwFaultErrorVector & ERROR_014){
    uwFaErVecLocal = ERROR_014;
  }
  else if(uwFaultErrorVector & ERROR_015){
    uwFaErVecLocal = ERROR_015;
  }
  else if(uwFaultErrorVector & ERROR_016){
    uwFaErVecLocal = ERROR_016;
  }  
  // ADD new Error Index
  
}

return uwFaErVecLocal;  
}




/**
  * @brief  DPC_FLT_Faulterror_Set: Set the Fault/error occours, the PWM output is automatically disabled.
  *         
  * @param eFaulterror: Fault or error ocourse  
  *
  * @retval None
  *
  * @note Function valid for STM32G4xx and STM32F74x microconroller family   
  */
void DPC_FLT_Faulterror_Set(DPC_FAULTERROR_LIST_TypeDef eFaulterror){
  uwFaultErrorVector |= eFaulterror;                                                    /*!< Set fault/error in the faulterror vector*/
}



/**
  * @brief  DPC_FLT_Error_Reset: Reset the error occours 
  *         
  * @param eError: Error to reset.  
  *
  * @retval DPC_FAULTERROR_LIST_TypeDef: error or fault occours, hiest priority
  *
  * @note Function valid for STM32G4xx and STM32F74x microconroller family   
  */
DPC_FAULTERROR_STATUS_TypeDef DPC_FLT_Error_Reset(DPC_FAULTERROR_LIST_TypeDef eError){

DPC_FAULTERROR_STATUS_TypeDef uwErVecLocal=NO_FAULTERROR;

if(uwFaultErrorVector & 0x0000FFFF){
  uwErVecLocal = NOT_ERASABLE;
}
else{
   if(uwFaultErrorVector & ERROR_PLL){
      uwFaultErrorVector &= !ERROR_PLL;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_IDLE){
      uwFaultErrorVector &= !ERROR_IDLE;
      uwErVecLocal = ERASE_OK;
    }            
    else if(uwFaultErrorVector & ERROR_START_INRS){
      uwFaultErrorVector &= !ERROR_START_INRS;
      uwErVecLocal = ERASE_OK;
    }           
    else if(uwFaultErrorVector & ERROR_FSM){
      uwFaultErrorVector &= !ERROR_FSM;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_PFC_UVLO){
      uwFaultErrorVector &= !ERROR_PFC_UVLO;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_BRS){
      uwFaultErrorVector &= !ERROR_BRS;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_AC_UV){
      uwFaultErrorVector &= !ERROR_AC_UV;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_PLL_OR){
      uwFaultErrorVector &= !ERROR_PLL_OR;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_PFC_RUN){
      uwFaultErrorVector &= !ERROR_PFC_RUN;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_AC_UVLO){
      uwFaultErrorVector &= !ERROR_AC_UVLO;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_AC_OFF){
      uwFaultErrorVector &= !ERROR_AC_OFF;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_PFC){
      uwFaultErrorVector &= !ERROR_PFC;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_013){
      uwFaultErrorVector &= !ERROR_013;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_014){
      uwFaultErrorVector &= !ERROR_014;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_015){
      uwFaultErrorVector &= !ERROR_015;
      uwErVecLocal = ERASE_OK;
    }
    else if(uwFaultErrorVector & ERROR_016){
      uwFaultErrorVector &= !ERROR_016;
      uwErVecLocal = ERASE_OK;
    }   
   if(uwFaultErrorVector | 0x00000000){
    uwErVecLocal = ERASE_ERROR_LIST_NOT_EMPTY;
  }
}
return uwErVecLocal;  
}





















