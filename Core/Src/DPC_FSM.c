/**
******************************************************************************
* @file           : FSM.c
* @brief          : Finite State Machine of Application Module
******************************************************************************
** 
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
#include "DPC_FSM.h"
#include "DPC_Faulterror.h"
/* Private variables ---------------------------------------------------------*/
static volatile DPC_FSM_State_t DPC_FSM_State;              /**< DSMPS state variable */
static volatile DPC_FSM_State_t DPC_FSM_NEW_State;              /**< DSMPS state variable */
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/**
  * @brief  Executes converter's state machine task
  * @param  None
  * @retval None
  */
void DPC_FSM_Application(void)
{    
    /**### Global State Machine */
  switch(DPC_FSM_State){                     
    case DPC_FSM_WAIT:    /** @arg  \a DP_FSM_WAIT state:  ...*/
      //if(checkfault.....  
         if(DPC_FSM_WAIT_Func()){
           DPC_FSM_State = DPC_FSM_State_Get();
         }
         else{
          DPC_FSM_State = DPC_FSM_STOP;
         }
      break;    
    case DPC_FSM_IDLE:    /** @arg  \a DP_FSM_IDLE state:  ... */            
        if(DPC_FSM_IDLE_Func()){
           DPC_FSM_State = DPC_FSM_State_Get();
        }
        else{
         DPC_FSM_State = DPC_FSM_STOP;
        }
      break;
    case DPC_FSM_INIT:    /** @arg  \a DP_FSM_INIT state: ... */
        
        if(DPC_FSM_INIT_Func()){
           DPC_FSM_State = DPC_FSM_State_Get();
        }
        else{
         DPC_FSM_State = DPC_FSM_STOP;
        }
      break;      
    case DPC_FSM_START:     /**  @arg \a DP_FSM_START state: ... */
        
        if(DPC_FSM_START_Func()){
           DPC_FSM_State = DPC_FSM_State_Get();
        }
        else{
         DPC_FSM_State = DPC_FSM_STOP;
        }
      break;  
    case DPC_FSM_RUN:           /** @arg \a DP_FSM_RUN state: ... */  
        
        if(DPC_FSM_RUN_Func()){
           DPC_FSM_State = DPC_FSM_State_Get();
        }
        else{
         DPC_FSM_State = DPC_FSM_STOP;
        }
      break;  
    case DPC_FSM_STOP:          /** @arg \a DP_FSM_STOP state: ... */
        
        if(DPC_FSM_STOP_Func()){
           DPC_FSM_State = DPC_FSM_State_Get();
        }
        else{
         DPC_FSM_State = DPC_FSM_ERROR;
        }
      break;
    case DPC_FSM_ERROR:         /** @arg \a DP_FSM_FAULT state: ... */
      
        if(DPC_FSM_ERROR_Func()){          
           DPC_FSM_State = DPC_FSM_FAULT;
        }
        else{
         DPC_FSM_State = DPC_FSM_FAULT;
        }
      break;
    case DPC_FSM_FAULT:         /** @arg \a DP_FSM_FAULT state: ... */
       DPC_FSM_FAULT_Func();
      break;
    default:
    break;
  }
  /*-- end of switch-case State Machine --*/ 
}

/**
  * @brief  Executes converter's state machine WAIT STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_WAIT_Func(void)
{
bool RetVal = true; 
 
  DPC_FSM_State_Set(DPC_FSM_IDLE);  
 
 return RetVal;
}

/**
  * @brief  Executes converter's state machine IDLE STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_IDLE_Func(void)
{
bool RetVal = true;   
 
  DPC_FSM_State_Set(DPC_FSM_INIT);
  
 return RetVal;
}
      
/**
  * @brief  Executes converter's state machine INIT STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_INIT_Func(void)
{
bool RetVal = true;   

  DPC_FSM_State_Set(DPC_FSM_START);
  
 return RetVal;
}

/**
  * @brief  Executes converter's state machine START STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_START_Func(void)
{
bool RetVal = true;   
 
  DPC_FSM_State_Set(DPC_FSM_RUN);
  
 return RetVal;
}


/**
  * @brief  Executes converter's state machine RUN STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_RUN_Func(void)
{
bool RetVal = true;

  DPC_FSM_State_Set(DPC_FSM_STOP);   
 
 return RetVal;
}


/**
  * @brief  Executes converter's state machine STOP STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_STOP_Func(void)
{
bool RetVal = true;   

  DPC_FSM_State_Set(DPC_FSM_ERROR);
 
 return RetVal;
}

/**
  * @brief  Executes converter's state machine ERR/FAUL STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_ERROR_Func(void)
{
bool RetVal = true;   
 
 return RetVal;
}

/**
  * @brief  Executes converter's state machine FAULT STate Function
  * @param  None
  * @retval true/false
  */
__weak bool DPC_FSM_FAULT_Func(void)
{
bool RetVal = true;   
 
 return RetVal;
}

/**
  * @brief  DPC_FSM_State_Get: return the new state of FSM
  * @param  None
  * @retval DP_FSM_State_t: new state
  */
DPC_FSM_State_t DPC_FSM_State_Get(void)
{   
 return DPC_FSM_NEW_State;
}


/**
  * @brief  DPC_FSM_State_Set: Set the new state of FSM
  * @param  DP_FSM_State_t: new state to set
  * @retval none
  */
void DPC_FSM_State_Set(DPC_FSM_State_t eNewStateVal)
{   
  DPC_FSM_NEW_State = eNewStateVal;
}