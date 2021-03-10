/**
  ******************************************************************************
  * @file           : DPC_PWMConverter.c
  * @brief          : PWM modulation management
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

//#ifdef USE_ADVTIM
//#include "tim.h"
//#endif


#include "hrtim.h"


#include "main.h"
#include "math.h"

#include "DPC_Math.h"
#include "DPC_PWMConverter.h"
  

/* external variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

////
float DT_dist_rangeA;              /*!< Distance to */
float DT_dist_rangeB;
float DT_dist_rangeC;
float DT_dist_rangeD;

float t_dtg_rangeA;
float t_dtg_rangeB;
float t_dtg_rangeC;
float t_dtg_rangeD;
float t_DTS;


//float t_tim_ket_ck;



/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  DPC_PWM_InitDUTY: Initialize the PULSE value of TIMn1 e TIMn2 used to do the PWM of Converter  
  *         TIMn1, TIMn2 and PWM_CHANNEL_x, must be selected in the DPC_Lib_Conf.f
  * @param uint16_t  (PULSE1_VALUE,PULSE2_VALUE,PULSE3_VALUE), value of Duty expressed in
  *        ARR ratio.   
  *
  * @retval Null
  *
  * @note Function valid for STM32G4xx family   
  */
void DPC_PWM_InitDUTY(uint16_t  PULSE1_VALUE,uint16_t  PULSE2_VALUE,uint16_t  PULSE3_VALUE){
    /* Common configuration for all channels */
#ifdef DPC_PWM_1ADVTIM_3CH
TIM_OC_InitTypeDef     sPWMConfig;

/* Set the pulse value for channel 1 */
  sPWMConfig.Pulse = PULSE1_VALUE;  
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  } 
  /* Set the pulse value for channel 2 */
  sPWMConfig.Pulse = PULSE2_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }  
  /* Set the pulse value for channel 4 */
  sPWMConfig.Pulse = PULSE3_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
 // DPC_PWM_1ADVTIM_3CH  

#elif  defined(DPC_PWM_1ADVTIM_2CH_1CHX)

TIM_OC_InitTypeDef     sPWMConfig;

/* Set the pulse value for channel 1 */
  sPWMConfig.Pulse = PULSE1_VALUE;  
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  sPWMConfig.Pulse = PULSE2_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
//DPC_PWM_1ADVTIM_2CH_1CHX

#elif  defined(DPC_PWM_2ADVTIM_3CH_3CHX)
TIM_OC_InitTypeDef     sPWMConfig;

/* Set the pulse value for channel 1 */
  sPWMConfig.Pulse = PULSE1_VALUE;  
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim2, &sPWMConfig, PWM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Set the pulse value for channel 2 */
  sPWMConfig.Pulse = PULSE2_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim2, &sPWMConfig, PWM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Set the pulse value for channel 3 */
  sPWMConfig.Pulse = PULSE3_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim1, &sPWMConfig, PWM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  if(HAL_TIM_PWM_ConfigChannel(&PWM_Tim2, &sPWMConfig, PWM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
//DPC_PWM_2ADVTIM_3CH_3CHX   

#endif 
}




/**
  * @brief  DPC_PWM_ADVTIM_PWMStop: Stop all the PWM of converter 
  *         TIMn1, TIMn2 and PWM_CHANNEL_x, must be selected in the DPC_Lib_Conf.f
  *
  * @param  Null
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx family   
  */
void DPC_PWM_ADVTIM_PWMStop(void){
  
#ifdef DPC_PWM_1ADVTIM_3CH
 
  /* ----------------------------------------PWM_Tim1 CH1---------------------------------------------- */
  /* Stop channel 1 */
   if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH2---------------------------------------------- */
  /* Stop channel 2 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_2) != HAL_OK)
  {   
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH3---------------------------------------------- */
  /* Stop channel 4 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_3) != HAL_OK)
  {  
    Error_Handler();
  }  
//DPC_PWM_1ADVTIM_3CH 

#elif defined(DPC_PWM_1ADVTIM_2CH_1CHX)
 
  /* ----------------------------------------PWM_Tim1 CH1---------------------------------------------- */
  /* Stop channel 1 */
   if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH2---------------------------------------------- */
  /* Stop channel 2 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_2) != HAL_OK)
  {   
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH1N---------------------------------------------- */
  /* Stop channel 4 */
 if(HAL_TIMEx_PWMN_Stop(&PWM_Tim1, PWM_CHANNEL_1N) != HAL_OK)
  {  
    Error_Handler();
  }
//DPC_PWM_1ADVTIM_2CH_1CHX   

#elif defined(DPC_PWM_2ADVTIM_3CH_3CHX) 
  /* ----------------------------------------PWM_Tim1 CH1-CH1N---------------------------------------------- */
  /* Stop channel 1 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Stop channel 1N */
  if(HAL_TIMEx_PWMN_Stop(&PWM_Tim1, PWM_CHANNEL_1N) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim2 CH1-CH1N---------------------------------------------- */
  if(HAL_TIM_PWM_Stop(&PWM_Tim2, PWM_CHANNEL_1) != HAL_OK)
  {  
    Error_Handler();
  }
  /* Stop channel 1N */
  if(HAL_TIMEx_PWMN_Stop(&PWM_Tim2, PWM_CHANNEL_1N) != HAL_OK)
  {  
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH2-CH2N---------------------------------------------- */
  /* Stop channel 2 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_2) != HAL_OK)
  {   
    Error_Handler();
  }
  /* Stop channel 2N */
  if(HAL_TIMEx_PWMN_Stop(&PWM_Tim1, PWM_CHANNEL_2N) != HAL_OK)
  {   
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim2 CH2-CH2N---------------------------------------------- */
  /* Stop channel 2 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim2, PWM_CHANNEL_2) != HAL_OK)
  { 
    Error_Handler();
  }
  /* Stop channel 2N */
  if(HAL_TIMEx_PWMN_Stop(&PWM_Tim2, PWM_CHANNEL_2N) != HAL_OK)
  {
    
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH3-CH3N---------------------------------------------- */
  /* Stop channel 3 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim1, PWM_CHANNEL_3) != HAL_OK)
  {  
    Error_Handler();
  }
  /* Stop channel 3N */
  if(HAL_TIMEx_PWMN_Stop(&PWM_Tim1, PWM_CHANNEL_3N) != HAL_OK)
  { 
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim2 CH3-CH3N---------------------------------------------- */
  /* Stop channel 3 */
  if(HAL_TIM_PWM_Stop(&PWM_Tim2, PWM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* Stop channel 3N */
  if(HAL_TIMEx_PWMN_Stop(&PWM_Tim2, PWM_CHANNEL_3N) != HAL_OK)
  {
    Error_Handler();
  }
//DPC_PWM_2ADVTIM_3CH_3CHX
#else

  return;

#endif  
}


/**
  * @brief  PWM_: Start all the PWM of converter 
  *         PWM_Timx and PWM_CHANNEL_x, must be selected in the DPC_Lib_Conf.f
  *
  * @param  Null
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx family   
  */
void DPC_PWM_ADVTIM_PWMStart(void){
  
#ifdef DPC_PWM_1ADVTIM_3CH
     
   /* ----------------------------------------PWM_Tim1 CH1---------------------------------------------- */
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH2---------------------------------------------- */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_2) != HAL_OK)
  {  
    Error_Handler();
  }

  /* ----------------------------------------PWM_Tim1 CH3---------------------------------------------- */
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, TIM_CHANNEL_3) != HAL_OK)
  {  
    Error_Handler();
  }

//DPC_PWM_1ADVTIM_3CH
  
#elif   defined(DPC_PWM_2ADVTIM_3CH_3CHX)    
  
  /* ----------------------------------------PWM_Tim1 CH1-CH1N---------------------------------------------- */
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Start channel 1N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim1, PWM_CHANNEL_1N) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim2 CH1-CH1N---------------------------------------------- */ 
  if(HAL_TIM_PWM_Start(&PWM_Tim2, PWM_CHANNEL_1) != HAL_OK)
  {   
    Error_Handler();
  }
  /* Start channel 1N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim2, PWM_CHANNEL_1N) != HAL_OK)
  {    
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH2-CH2N---------------------------------------------- */
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_2) != HAL_OK)
  {   
    Error_Handler();
  }
  /* Start channel 2N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim1, PWM_CHANNEL_2N) != HAL_OK)
  {  
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim2 CH2-CH2N---------------------------------------------- */
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&PWM_Tim2, PWM_CHANNEL_2) != HAL_OK)
  {   
    Error_Handler();
  }
  /* Start channel 2N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim2, PWM_CHANNEL_2N) != HAL_OK)
  {   
    Error_Handler();
  } 
  /* ----------------------------------------PWM_Tim1 CH3-CH3N---------------------------------------------- */
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_3) != HAL_OK)
  {   
    Error_Handler();
  }
  /* Start channel 3N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim1, PWM_CHANNEL_3N) != HAL_OK)
  {  
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim2 CH3-CH3N---------------------------------------------- */
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&PWM_Tim2, PWM_CHANNEL_3) != HAL_OK)
  {  
    Error_Handler();
  }
  /* Start channel 3N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim2, PWM_CHANNEL_3N) != HAL_OK)
  {   
    Error_Handler();
  }
//DPC_PWM_2ADVTIM_3CH_3CHX  
#elif   defined(DPC_PWM_1ADVTIM_2CH_1CHX)
 /* ----------------------------------------PWM_Tim1 CH1-CH1N---------------------------------------------- */
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Start channel 1N */
  if(HAL_TIMEx_PWMN_Start(&PWM_Tim1, PWM_CHANNEL_1N) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------------------------PWM_Tim1 CH2---------------------------------------------- */
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&PWM_Tim1, PWM_CHANNEL_2) != HAL_OK)
  {   
    Error_Handler();
  }


#else
 // Put here feature conf. of ADVTIM

#endif  
  
}



/**
  * @brief  DPC_PWM_ADVTIM_SET_DeadTime: Set the dead time of ADVTIM  
  *         T PWM_Timx and PWM_CHANNEL_x, must be selected in the DPC_Lib_Conf.f
  *
  * @param  uint16_t uwDeadTimeSub> value od dead time 
  *
  * @retval Null 
  *
  * @note This function can be use only if the Dead time upgrade is UNLOCKED, 
  *        see TIM Lock level in stm32g4xx_hal_tim.  
  * @note Function valid for STM32G4xx family   
  */
void DPC_PWM_ADVTIM_SET_DeadTime(uint16_t uwDeadTimeSub)
{
#ifdef DPC_PWM_2ADVTIM_3CH_3CHX  
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

sBreakDeadTimeConfig.DeadTime = uwDeadTimeSub;

  if (HAL_TIMEx_ConfigBreakDeadTime(&PWM_Tim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&PWM_Tim1);

  if (HAL_TIMEx_ConfigBreakDeadTime(&PWM_Tim2, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&PWM_Tim2);

//DPC_PWM_2ADVTIM_3CH_3CHX
  
#else
 // Put here feature conf. of ADVTIM

#endif     
}




/**
  * @brief  DPC_PWM_Send_Burst_PWM:
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void DPC_PWM_Send_Burst_PWM(DPC_PWM_TypeDef *tDPC_PWM_loc,float BURST_A,float BURST_B,float BURST_C,DMA_PWMDUTY_STRUCT* DMA_SRC ){

#ifdef STDES_PFCBIDIR

    #ifdef DPC_PWM_1ADVTIM_3CH


//DPC_PWM_1ADVTIM_3CH

#elif   defined(DPC_PWM_2ADVTIM_3CH_3CHX)
  //_______________________
  uint16_t PWM_PERIOD_COUNTER_INT;

  PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&PWM_Tim1);                                    /*!< Check and upgrade if Burst period is correct >*/
  if (PWM_PERIOD_COUNTER_INT!=tDPC_PWM_loc->PWM_Period){
    DPC_PWM_OutDisable();
    __HAL_TIM_SET_AUTORELOAD(&PWM_Tim1, tDPC_PWM_loc->PWM_Period);
    DPC_PWM_OutEnable(tDPC_PWM_loc);
  }

  PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&PWM_Tim2);                                    /*!< Check and upgrade if Burst period is correct >*/
  if (PWM_PERIOD_COUNTER_INT!=tDPC_PWM_loc->PWM_Period){
    DPC_PWM_OutDisable();
    __HAL_TIM_SET_AUTORELOAD(&PWM_Tim2, tDPC_PWM_loc->PWM_Period);
    DPC_PWM_OutEnable(tDPC_PWM_loc);
  }

  PWM_PERIOD_COUNTER_INT=tDPC_PWM_loc->PWM_Period;                                              /*!< load period  >*/

    PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&htim1);                                    ///Future improuvments: USE MACRO INSTEAD PWM_PERIOD_COUNTER_INT

  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)((1.0f-BURST_A)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)((1.0f-BURST_B)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)((1.0f-BURST_C)*PWM_PERIOD_COUNTER_INT));

  __HAL_TIM_SET_COMPARE(&PWM_Tim2, PWM_CHANNEL_1, (uint32_t)((1.0f-BURST_A)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim2, PWM_CHANNEL_2, (uint32_t)((1.0f-BURST_B)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim2, PWM_CHANNEL_3, (uint32_t)((1.0f-BURST_C)*PWM_PERIOD_COUNTER_INT));

//DPC_PWM_2ADVTIM_3CH_3CHX


#elif   defined(DPC_PWM_1HRTIM_3CH)
/*             _______WIP_____
  uint16_t ValueH;
  uint16_t ValueL;
  uint16_t Period;
  Period=__HAL_HRTIM_GETPERIOD(&hhrtim1, 0x0);
  ValueH =(uint16_t)( Period *  Duty);
  ValueL =(uint16_t)( Period *  (float)(1.0-Duty));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValueL);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x2,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x3,HRTIM_COMPAREUNIT_1,ValueL);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x4,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x5,HRTIM_COMPAREUNIT_1,ValueL);
            _______WIP_____
*/

#else



#endif


#elif STDES_PFCBIDIR_REV2

    #ifdef DPC_PWM_1ADVTIM_3CH


//DPC_PWM_1ADVTIM_3CH

#elif   defined(DPC_PWM_2ADVTIM_3CH_3CHX)
    uint16_t PWM_PERIOD_COUNTER_INT;
    PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&htim1);   ///Future improuvments: USE MACRO INSTEAD PWM_PERIOD_COUNTER_INT

  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)((1.0f-BURST_A)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)((1.0f-BURST_B)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)((1.0f-BURST_C)*PWM_PERIOD_COUNTER_INT));

  __HAL_TIM_SET_COMPARE(&PWM_Tim2, PWM_CHANNEL_1, (uint32_t)((1.0f-BURST_A)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim2, PWM_CHANNEL_2, (uint32_t)((1.0f-BURST_B)*PWM_PERIOD_COUNTER_INT));
  __HAL_TIM_SET_COMPARE(&PWM_Tim2, PWM_CHANNEL_3, (uint32_t)((1.0f-BURST_C)*PWM_PERIOD_COUNTER_INT));

//DPC_PWM_2ADVTIM_3CH_3CHX

#elif   defined(DPC_PWM_1HRTIM_3CH)
/*             _______WIP_____
  uint16_t ValueH;
  uint16_t ValueL;
  uint16_t Period;
  Period=__HAL_HRTIM_GETPERIOD(&hhrtim1, 0x0);
  ValueH =(uint16_t)( Period *  Duty);
  ValueL =(uint16_t)( Period *  (float)(1.0-Duty));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValueL);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x2,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x3,HRTIM_COMPAREUNIT_1,ValueL);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x4,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x5,HRTIM_COMPAREUNIT_1,ValueL);
            _______WIP_____
*/

#else



#endif


#elif STDES_VIENNARECT

    #ifdef DPC_PWM_1ADVTIM_3CH

    uint16_t PWM_PERIOD_COUNTER_INT;
    PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&htim1, 0x0);   ///Future improuvments: USE MACRO INSTEAD PWM_PERIOD_COUNTER_INT


  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)BURST_B*PWM_PERIOD_COUNTER_INT);
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)BURST_C*PWM_PERIOD_COUNTER_INT);
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)BURST_A*PWM_PERIOD_COUNTER_INT);


//DPC_PWM_1ADVTIM_3CH

#elif   defined(DPC_PWM_2ADVTIM_3CH_3CHX)


    uint16_t PWM_PERIOD_COUNTER_INT;
    PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&htim1);   ///Future improuvments: USE MACRO INSTEAD PWM_PERIOD_COUNTER_INT



  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)BURST_A*PWM_PERIOD_COUNTER_INT);
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)BURST_B*PWM_PERIOD_COUNTER_INT);
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)BURST_C*PWM_PERIOD_COUNTER_INT);

  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)BURST_A*PWM_PERIOD_COUNTER_INT);
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)BURST_B*PWM_PERIOD_COUNTER_INT);
  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)BURST_C*PWM_PERIOD_COUNTER_INT);

//DPC_PWM_2ADVTIM_3CH_3CHX


#elif   defined(DPC_PWM_1HRTIM_3CH)
  uint16_t ValuePhA;
  uint16_t ValuePhB;
  uint16_t ValuePhC;
  uint16_t Period;
  Period=__HAL_HRTIM_GETPERIOD(&hhrtim1, 0x0);
  ValuePhA =(uint16_t)((1.0f-BURST_A)*Period);
  ValuePhB =(uint16_t)((1.0f-BURST_B)*Period);
  ValuePhC =(uint16_t)((1.0f-BURST_C)*Period);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValuePhB);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_2,ValuePhC);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValuePhA);
#else


#endif

#else
  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION

#endif


  uint32_t dutyVApos;
  uint32_t dutyVAneg;
  uint32_t dutyVBpos;
  uint32_t dutyVBneg;
  uint32_t dutyVCpos;
  uint32_t dutyVCneg;

  uint16_t PWM_PERIOD_COUNTER_INT;

  PWM_PERIOD_COUNTER_INT=__HAL_HRTIM_GETPERIOD(&PWM_Tim1, HRTIM_TIMERINDEX_TIMER_A);
  dutyVApos = BURST_A*PWM_PERIOD_COUNTER_INT;
  dutyVBpos = BURST_B*PWM_PERIOD_COUNTER_INT;
  dutyVCpos = BURST_C*PWM_PERIOD_COUNTER_INT;


	DMA_SRC->phAA=dutyVApos;
	DMA_SRC->phAB=dutyVApos;
	DMA_SRC->phBA=dutyVBpos;
	DMA_SRC->phBB=dutyVBpos;
	DMA_SRC->phCA=dutyVCpos;
	DMA_SRC->phCB=dutyVCpos;
	DMA_SRC->phA=dutyVApos;
	DMA_SRC->phB=dutyVBpos;
	DMA_SRC->phC=dutyVCpos;

//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValueH);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValueL);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x2,HRTIM_COMPAREUNIT_1,ValueH);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x3,HRTIM_COMPAREUNIT_1,ValueL);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x4,HRTIM_COMPAREUNIT_1,ValueH);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x5,HRTIM_COMPAREUNIT_1,ValueL);
  
}


/**
* @brief  
*         
* @param  
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_Send_Duty_SPWM(DPC_PWM_TypeDef *tDPC_PWM_loc,float VA,float VB,float VC, DMA_PWMDUTY_STRUCT* DMA_SRC)
{
  
  if(VA>1){VA=1;}
  else if(VA<-1){VA=-1;}
  if(VB>=1){VB=1;}
  else if(VB<-1){VB=-1;}
  if(VC>=1){VC=1;}
  else if(VC<-1){VC=-1;}  

  uint16_t PWM_PERIOD_COUNTER_INT;
  PWM_PERIOD_COUNTER_INT=__HAL_HRTIM_GETPERIOD(&PWM_Tim1, HRTIM_TIMERINDEX_TIMER_A);
  
  float VApos;
  float VAneg;
  float VBpos;
  float VBneg;
  float VCpos;
  float VCneg;
  
  uint32_t dutyVApos;
  uint32_t dutyVAneg;
  uint32_t dutyVBpos;
  uint32_t dutyVBneg;
  uint32_t dutyVCpos;
  uint32_t dutyVCneg;  
  
  if(VA>=0.0f){VApos=VA;VAneg=0;}else if(VA<0.0f){VApos=0.0f;VAneg=-1*VA;}
  if(VB>=0.0f){VBpos=VB;VBneg=0;}else if(VB<0.0f){VBpos=0.0f;VBneg=-1*VB;}
  if(VC>=0.0f){VCpos=VC;VCneg=0;}else if(VC<0.0f){VCpos=0.0f;VCneg=-1*VC;}

  if(VA>=0.0f){VApos=VA;VAneg=0;}else if(VA<0.0f){VApos=-1*VA;VAneg=-1*VA;}
  if(VB>=0.0f){VBpos=VB;VBneg=0;}else if(VB<0.0f){VBpos=-1*VB;VBneg=-1*VB;}
  if(VC>=0.0f){VCpos=VC;VCneg=0;}else if(VC<0.0f){VCpos=-1*VC;VCneg=-1*VC;}




  tDPC_PWM_loc->VApos=(float) (1.0f-VApos);
  tDPC_PWM_loc->VBpos=VBpos;
  tDPC_PWM_loc->VCpos=VCpos;
  tDPC_PWM_loc->VAneg=VAneg;
  tDPC_PWM_loc->VBneg=VBneg;
  tDPC_PWM_loc->VCneg=VCneg;
  
  dutyVApos=(uint32_t)(tDPC_PWM_loc->VApos*PWM_PERIOD_COUNTER_INT);
  dutyVAneg=(uint32_t)(tDPC_PWM_loc->VAneg*PWM_PERIOD_COUNTER_INT);
  
  dutyVBpos=(uint32_t)(tDPC_PWM_loc->VBpos*PWM_PERIOD_COUNTER_INT);
  dutyVBneg=(uint32_t)(tDPC_PWM_loc->VBneg*PWM_PERIOD_COUNTER_INT);
  
  dutyVCpos=(uint32_t)(tDPC_PWM_loc->VCpos*PWM_PERIOD_COUNTER_INT);
  dutyVCneg=(uint32_t)(tDPC_PWM_loc->VCneg*PWM_PERIOD_COUNTER_INT);
  
  if(dutyVApos>=tDPC_PWM_loc->dutyMaxLim){dutyVApos=tDPC_PWM_loc->dutyMaxLim;}
  else if(dutyVApos<tDPC_PWM_loc->dutyMinLim){dutyVApos=tDPC_PWM_loc->dutyMinLim;}
  
  if(dutyVAneg>=tDPC_PWM_loc->dutyMaxLim){dutyVAneg=tDPC_PWM_loc->dutyMaxLim;}
  else if(dutyVAneg<tDPC_PWM_loc->dutyMinLim){dutyVAneg=tDPC_PWM_loc->dutyMinLim;}
  
  if(dutyVBpos>=tDPC_PWM_loc->dutyMaxLim){dutyVBpos=tDPC_PWM_loc->dutyMaxLim;}
  else if(dutyVBpos<tDPC_PWM_loc->dutyMinLim){dutyVBpos=tDPC_PWM_loc->dutyMinLim;}
  
  if(dutyVBneg>tDPC_PWM_loc->dutyMaxLim){dutyVBneg=tDPC_PWM_loc->dutyMaxLim;}
  else if(dutyVBneg<tDPC_PWM_loc->dutyMinLim){dutyVBneg=tDPC_PWM_loc->dutyMinLim;}
  
  if(dutyVCpos>=tDPC_PWM_loc->dutyMaxLim){dutyVCpos=tDPC_PWM_loc->dutyMaxLim;}
  else if(dutyVCpos<tDPC_PWM_loc->dutyMinLim){dutyVCpos=tDPC_PWM_loc->dutyMinLim;}
  
  if(dutyVCneg>=tDPC_PWM_loc->dutyMaxLim){dutyVCneg=tDPC_PWM_loc->dutyMaxLim;}
  else if(dutyVCneg<tDPC_PWM_loc->dutyMinLim){dutyVCneg=tDPC_PWM_loc->dutyMinLim;}   

 // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,HRTIM_COMPAREUNIT_1,dutyVApos);
//  if (dutyVApos<=500){
//	  //HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TA1);
//	  dutyVApos=0;
//  }
  //else HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TA1);
  	DMA_SRC->phAA=dutyVApos;
  	DMA_SRC->phAB=dutyVAneg;
  	DMA_SRC->phBA=dutyVBpos;
  	DMA_SRC->phBB=dutyVBneg;
  	DMA_SRC->phCA=dutyVCpos;
  	DMA_SRC->phCB=dutyVCneg;
  	DMA_SRC->phA=dutyVApos;
  	DMA_SRC->phB=dutyVBpos+dutyVBneg;
  	DMA_SRC->phC=dutyVCpos+dutyVCneg;



  	//DMA_SRC[0]=5000;
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,HRTIM_COMPAREUNIT_1,dutyVBpos);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,HRTIM_COMPAREUNIT_1,dutyVCpos);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,HRTIM_COMPAREUNIT_1,dutyVAneg);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,HRTIM_COMPAREUNIT_1,dutyVBneg);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,HRTIM_COMPAREUNIT_1,dutyVCneg);
      
  
  //DPC_PWM_1HRTIM_6CH_6CHX   
  
//#else
  
  
//#endif
  
//#elif STDES_VIENNARECT
//
//#ifdef DPC_PWM_1ADVTIM_3CH
//
//  uint16_t PWM_PERIOD_COUNTER_INT;
//  PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&htim1, 0x0);   ///Future improuvments: USE MACRO INSTEAD PWM_PERIOD_COUNTER_INT
//
//
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)VB*PWM_PERIOD_COUNTER_INT);
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)VC*PWM_PERIOD_COUNTER_INT);
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)VA*PWM_PERIOD_COUNTER_INT);
//
//
//  //DPC_PWM_1ADVTIM_3CH
//
//#elif   defined(DPC_PWM_2ADVTIM_3CH_3CHX)
//
//  uint16_t PWM_PERIOD_COUNTER_INT;
//  PWM_PERIOD_COUNTER_INT=__HAL_TIM_GET_AUTORELOAD(&htim1);   ///Future improuvments: USE MACRO INSTEAD PWM_PERIOD_COUNTER_INT
//
//  float VApos;
//  float VAneg;
//  float VBpos;
//  float VBneg;
//  float VCpos;
//  float VCneg;
//
//  if(VA>=0.0f){VApos=VA;VAneg=0;}else if(VA<0.0f){VApos=0.0f;VAneg=-1*VA;}
//  if(VB>=0.0f){VBpos=VB;VBneg=0;}else if(VB<0.0f){VBpos=0.0f;VBneg=-1*VB;}
//  if(VC>=0.0f){VCpos=VC;VCneg=0;}else if(VC<0.0f){VCpos=0.0f;VCneg=-1*VC;}
//
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)VApos*PWM_PERIOD_COUNTER_INT);
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)VBpos*PWM_PERIOD_COUNTER_INT);
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)VCpos*PWM_PERIOD_COUNTER_INT);
//
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_1, (uint32_t)VAneg*PWM_PERIOD_COUNTER_INT);
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_2, (uint32_t)VBneg*PWM_PERIOD_COUNTER_INT);
//  __HAL_TIM_SET_COMPARE(&PWM_Tim1, PWM_CHANNEL_3, (uint32_t)VCneg*PWM_PERIOD_COUNTER_INT);
//
//  //DPC_PWM_2ADVTIM_3CH_3CHX
//#elif   defined(DPC_PWM_1HRTIM_3CH)
//  uint16_t ValuePhA;
//  uint16_t ValuePhB;
//  uint16_t ValuePhC;
//  uint16_t Period;
//  Period=__HAL_HRTIM_GETPERIOD(&hhrtim1, 0x0);
//  ValuePhA =(uint16_t)(VA*Period);
//  ValuePhB =(uint16_t)(VB*Period);
//  ValuePhC =(uint16_t)(VC*Period);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValuePhB);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_2,ValuePhC);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValuePhA);
//  //DPC_PWM_1HRTIM_3CH
//
//#else
//#endif
//#else
//  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION
//#endif
//    return;
}







/**
  * @brief  DPC_Calc_DTG: Accordingly with DT input variable expressed in seconds provide a better DTG[7:0] configuration of TIMx_BDTR
  * 
  * @param  DT_TimeVal: Dead time value expressed in seconds
  * @retval DTG_bin: DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR)   
  * @note  Function linked to  DPC_PWM_Calc_DeadTimRange function  
  * @note  Function valid for STM32G474 microconroller   
  * @note  See 27.6.20 of RM0440 Rev1 (pag1162/2083)   
  * @note  RangeX
            RangeA -> DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
            RangeB -> DTG[7:5]=10x => DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
            RangeC -> DTG[7:5]=110 => DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS.
            RangeD -> DTG[7:5]=111 => DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS.
  * @warning LOCK config must be disable to obtain effect of DTG_bin Setting
  */  
uint8_t DPC_Calc_DTG(float DT_TimeVal)
{
  uint32_t DTG_dec;             /*!< DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR) expressed in decimal integer*/  
  uint32_t DTG_bin;             /*!< DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR) expressed in HEX*/ 
  float DT_dist_range_min;         /*!< Local variable to save the dist_rangeX if it's lower*/
  
  
  /*! Find lower dist_rangeX*/
  
  if(DT_dist_rangeA<DT_dist_rangeB){
    DT_dist_range_min=DT_dist_rangeA;
  }
  else
  {
    DT_dist_range_min=DT_dist_rangeB;
  }
  
  if(DT_dist_rangeB<DT_dist_rangeC)
  {
    DT_dist_range_min=DT_dist_range_min;
  }
  else{
    DT_dist_range_min=DT_dist_rangeC;
  }
  if(DT_dist_rangeC<DT_dist_rangeD)
  {
    DT_dist_range_min=DT_dist_range_min;
  }
  else{
    DT_dist_range_min=DT_dist_rangeD;
  }
  
  
  /*! Generate DTG[7-5] according with the better range*/ 
  
  if (DT_dist_range_min==DT_dist_rangeA){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeA));
    DTG_bin = (DTG_dec & 0x7F) | 0x00;                  /// DTG[7-5]=0xx <=> *0x7F=0111 1111 e 0x00=0000 0000
  }
  else if (DT_dist_range_min==DT_dist_rangeB){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeB))-64;
    DTG_bin = (DTG_dec & 0x3F) | 0x80;                  /// DTG[7-5]=10x <=> *0x3F=0011 1111 e 0x80=1000 0000
  } 
  else if (DT_dist_range_min==DT_dist_rangeC){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeC))-32;
    DTG_bin = (DTG_dec & 0x1F) | 0xC0;                  /// DTG[7-5]=110 <=> *0x1F=0001 1111 e  0xC0=1100 0000 
  }
  else if (DT_dist_range_min==DT_dist_rangeD){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeD))-32;
    DTG_bin = (DTG_dec & 0x1F) | 0xE0;                  /// DTG[7-5]=111 <=> *0x1F=0001 1111 e  0xE0=1110 0000   
  }
   
    return DTG_bin;
  }



/**
  * @brief  DPC_Calc_DTG_Range: Accordingly to TIMx clk , prescaler and DeadTime request the possible DeadTime ranges will be determinate.  
  * @param  t_tim_ket_ck: Internal clock (tim_ker_ck)
  * @param  DT_TimeVal: Dead time value expressed in seconds
  * @retval none
  * @note  Function linked to  DPC_Calc_DTG function  
  * @note  Function valid for STM32G474 microconroller   
  * @note  See 27.6.20 of RM0440 Rev1 (pag 1162/2083) & 27.6.1 TIMx control register 1 (TIMx_CR1)(x = 1, 8, 20) of RM0440 Rev1 (pag 1131/2083)
  * @note __HAL_TIM_GET_CLOCKDIVISION -> CKD[1:0]: Clock division This bit-field indicates the division ratio between the timer clock (tim_ker_ck) frequency and the dead-time and sampling clock (tDTS)used by the dead-time generators and the digital filters (tim_etr_in, tim_tix),
  *         
  */  
void DPC_Calc_DTG_Range(float DT_TimeVal, float t_tim_ket_ck){
  
  
//#ifdef USE_ADVTIM
///*!Get clock division of the PWM advanced timer to determinate the dead-time and sampling clock (tDTS)*/
// uint32_t Tim_Clock_Division_DivX=__HAL_TIM_GET_CLOCKDIVISION(&PWM_Tim1);
//
//  switch(Tim_Clock_Division_DivX){
//  case TIM_CLOCKDIVISION_DIV1:  //
//    t_DTS=t_tim_ket_ck;
//    break;
//  case TIM_CLOCKDIVISION_DIV2:  //
//    t_DTS=2*t_tim_ket_ck;
//    break;
//  case TIM_CLOCKDIVISION_DIV4:  //
//    t_DTS=4*t_tim_ket_ck;
//    break;
//  default:
//  Error_Handler();
//    break;
//  }
//
///*!
//Dead Time Range - DT=DTG[7:0]x tdtg with tdtg=tDTS
//*/
//  t_dtg_rangeA=t_DTS;
//  float DTG_rangeA_max=127.0*t_dtg_rangeA;
//  float DTG_rangeA_min=0.0*t_dtg_rangeA;
////  float step_rangeA=t_dtg_rangeA;
//  float center_rangeA=(DTG_rangeA_max+DTG_rangeA_min)/2;
//  DT_dist_rangeA=center_rangeA-DT_TimeVal;
//  if(DT_dist_rangeA<0){DT_dist_rangeA=-1.0f*DT_dist_rangeA;}else{DT_dist_rangeA=DT_dist_rangeA;}
//
///*!
//Dead Time Range - DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
//*/
//  t_dtg_rangeB=2.0*t_DTS;
//  float DTG_rangeB_max=(64+63.0)*t_dtg_rangeB;
//  float DTG_rangeB_min=(64+0.0)*t_dtg_rangeB;
////  float step_rangeB=t_dtg_rangeB;
//  float center_rangeB=(DTG_rangeB_max+DTG_rangeB_min)/2;
//  DT_dist_rangeB=center_rangeB-DT_TimeVal;
//  if(DT_dist_rangeB<0){DT_dist_rangeB=-1.0f*DT_dist_rangeB;}else{DT_dist_rangeB=DT_dist_rangeB;}
//
///*!
//Dead Time Range - DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS
//*/
//  t_dtg_rangeC=8.0*t_DTS;
//  float DTG_rangeC_max=(32+31.0)*t_dtg_rangeC;
//  float DTG_rangeC_min=(32+0.0)*t_dtg_rangeC;
////  float step_rangeC=t_dtg_rangeC;
//  float center_rangeC=(DTG_rangeC_max+DTG_rangeC_min)/2;
//  DT_dist_rangeC=center_rangeC-DT_TimeVal;
//  if(DT_dist_rangeC<0){DT_dist_rangeC=-1.0f*DT_dist_rangeC;}else{DT_dist_rangeC=DT_dist_rangeC;}
//
///*!
//Dead Time Range - DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS
//*/
//  t_dtg_rangeD=16.0*t_DTS;
//  float DTG_rangeD_max=(32+31.0)*t_dtg_rangeD;
//  float DTG_rangeD_min=(32+0.0)*t_dtg_rangeD;
////  float step_rangeD=t_dtg_rangeD;
//  float center_rangeD=(DTG_rangeD_max+DTG_rangeD_min)/2;
//  DT_dist_rangeD=center_rangeD-DT_TimeVal;
//  if(DT_dist_rangeD<0){DT_dist_rangeD=-1.0f*DT_dist_rangeD;}else{DT_dist_rangeD=DT_dist_rangeD;}
//
//#elif USE_HRTIM
//    //Put here the HRTIM Function
//#endif


 
}


/**
  * @brief  DPC_ConfigDeadTime: Set DTC configuration
  * @param  
  * @param  
  * @retval none
  * @note  Function linked to  DPC_Calc_DTG and DPC_Calc_DTG_Range functions   
  * @note  Function valid for STM32G474 microconroller   
  *         
  */ 
void DPC_ConfigDeadTime(TIM_HandleTypeDef *htim_HS_AS,TIM_HandleTypeDef *htim_LS_DS,uint32_t Deadtime) {
  
//#ifdef USE_ADVTIM
// HAL_TIMEx_ConfigDeadTime(htim_HS_AS, Deadtime);
 // HAL_TIMEx_ConfigDeadTime(htim_LS_DS, Deadtime);
//#elif USE_HRTIM
  //
//#else
 // SELECT DEFINE
//#endif
}




/**
* @brief  DPC_PWM_HRTIM_Set: Set the value of duty of PWM generated by HRTIM 
*         
* @param  Period, Duty value accepted 0 - 1
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
//#ifdef USE_HRTIM
void DPC_PWM_HRTIM_Set(uint32_t Period,float Duty)
{  
//#ifdef STDES_PFCBIDIR_REV2
  uint16_t ValueH;
  uint16_t ValueL;
  ValueH =(uint16_t)( Period *  Duty); 
  ValueL =(uint16_t)( Period *  (float)(1.0-Duty));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValueL);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x2,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x3,HRTIM_COMPAREUNIT_1,ValueL);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x4,HRTIM_COMPAREUNIT_1,ValueH);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x5,HRTIM_COMPAREUNIT_1,ValueL);  
  
//#elif   STDES_VIENNARECT
//  uint16_t ValueH;
//  uint16_t ValueL;
//  ValueH =(uint16_t)( Period *  Duty);
//  ValueL =(uint16_t)( Period *  (float)(1.0-Duty));
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValueH);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValueL);
//#else
//  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION
//
//#endif
    
}
//#endif


/**
* @brief  DPC_PWM_HRTIM_Stop: Stop PWM generated by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/

//#ifdef USE_HRTIM
void DPC_PWM_HRTIM_Stop(void)
{
//#ifdef STDES_PFCBIDIR_REV2
  HAL_HRTIM_WaveformCountStop(&PWM_Tim1,HRTIM_TIMERID_TIMER_A + HRTIM_TIMERID_TIMER_B + HRTIM_TIMERID_TIMER_C + HRTIM_TIMERID_TIMER_D + HRTIM_TIMERID_TIMER_E + HRTIM_TIMERID_TIMER_F);

}
//#endif


/**
* @brief  Start PWM generated by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/

void DPC_PWM_HRTIM_Start(void)
{     

//  HAL_HRTIM_WaveformCounterStart(&PWM_Tim1, HRTIM_TIMERID_TIMER_A + HRTIM_TIMERID_TIMER_B + HRTIM_TIMERID_TIMER_C + HRTIM_TIMERID_TIMER_D
//                                 + HRTIM_TIMERID_TIMER_E + HRTIM_TIMERID_TIMER_F);
//  HAL_HRTIM_WaveformOutputStart(&PWM_Tim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2 + HRTIM_OUTPUT_TB1 + HRTIM_OUTPUT_TB2 + HRTIM_OUTPUT_TC1 + HRTIM_OUTPUT_TC2 + + HRTIM_OUTPUT_TD1 + HRTIM_OUTPUT_TD2 + HRTIM_OUTPUT_TE1 + HRTIM_OUTPUT_TE2 + HRTIM_OUTPUT_TF1 + HRTIM_OUTPUT_TF2);

//	HAL_HRTIM_WaveformCounterStart(&PWM_Tim1, HRTIM_TIMERID_TIMER_A + HRTIM_TIMERID_TIMER_B + HRTIM_TIMERID_TIMER_C);

	HAL_HRTIM_WaveformCounterStart_DMA(&PWM_Tim1, HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCounterStart_DMA(&PWM_Tim1, HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformCounterStart_DMA(&PWM_Tim1, HRTIM_TIMERID_TIMER_C);
	HAL_HRTIM_WaveformCountStart_IT(&PWM_Tim1, HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCountStart_IT(&PWM_Tim1, HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformCountStart_IT(&PWM_Tim1, HRTIM_TIMERID_TIMER_C);
	//HAL_HRTIM_WaveformCountStart_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
	//HAL_HRTIM_WaveformCounterStart(&PWM_Tim1, HRTIM_TIMERID_TIMER_B);
	//HAL_HRTIM_WaveformCounterStart(&PWM_Tim1, HRTIM_TIMERID_TIMER_C);

	//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1, HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&PWM_Tim1, HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&PWM_Tim1, HRTIM_OUTPUT_TB1);
	HAL_HRTIM_WaveformOutputStart(&PWM_Tim1, HRTIM_OUTPUT_TC1);
}



/**
* @brief  DPC_PWM_HRTIM_Init: Start PWM generated by HRTIM 
*         
* @param  Period, Duty value accepted 0 - 1
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
//#ifdef USE_HRTIM
void DPC_PWM_HRTIM_Init(void)
{     
//#ifdef STDES_PFCBIDIR_REV2
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2 + HRTIM_OUTPUT_TB1 + HRTIM_OUTPUT_TB2 + HRTIM_OUTPUT_TC1 + HRTIM_OUTPUT_TC2
                                + HRTIM_OUTPUT_TD1 + HRTIM_OUTPUT_TD2 + HRTIM_OUTPUT_TE1 + HRTIM_OUTPUT_TE2 + HRTIM_OUTPUT_TF1 + HRTIM_OUTPUT_TF2);     
//#elif   STDES_VIENNARECT
//  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2 + HRTIM_OUTPUT_TB2);
//#else
//  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION
//#endif
}
//#endif






/**
* @brief  DPC_PWM_HRTIM_OutDisable Output DISABLE PWM generator by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_HRTIM_OutDisable(void)
{     
//#ifdef STDES_PFCBIDIR_REV2
HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TA1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TA2);
HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TB1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TB2);
HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TC1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TC2);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TD1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TD2);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TE1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TE2);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TF1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TF2);
//#elif   STDES_VIENNARECT
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TA1);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TA2);
//HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,HRTIM_OUTPUT_TB2);
//#else
////  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION
//#endif
}





/**
* @brief  DPC_PWM_HRTIM_OutEnable Output ENABLE PWM generator by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_HRTIM_OutEnable(void)
{     
//#ifdef STDES_PFCBIDIR_REV2
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TA1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TA2);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TB1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TB2);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TC1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TC2);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TD1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TD2);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TE1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TE2);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TF1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TF2);
//#elif   STDES_VIENNARECT
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TA1);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TA2);
//HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,HRTIM_OUTPUT_TB2);
//#else
////  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION
//#endif
}







/**
* @brief  DPC_PWM_HRTIM_OutDisable Output DISABLE PWM generator by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_ADVTIM_OutDisable(void)
{     
//#ifdef DPC_PWM_2ADVTIM_3CH_3CHX
//  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim1);
//  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim2);
//#endif
//
//#ifdef DPC_PWM_1ADVTIM_3CH
//  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim1);
//#endif
//
//#ifdef DPC_PWM_1ADVTIM_2CH_1CHX
//  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim1);
//#endif
//
  //#ifdef DPC_PWM_xADVTIM_x CH Future dev.  
  //      .... Put here the code 
  //#endif      
}





/**
* @brief  DPC_PWM_ADVTIM_OutEnable Output ENABLE PWM generator by ADVTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_ADVTIM_OutEnable(void)
{
//#ifdef DPC_PWM_2ADVTIM_3CH_3CHX
//  __HAL_TIM_MOE_ENABLE(&PWM_Tim1);
//  __HAL_TIM_MOE_ENABLE(&PWM_Tim2);
//#endif
//
//#ifdef DPC_PWM_1ADVTIM_3CH
//  __HAL_TIM_MOE_ENABLE(&PWM_Tim1);
//#endif
//
//#ifdef DPC_PWM_1ADVTIM_2CH_1CHX
//  __HAL_TIM_MOE_ENABLE(&PWM_Tim1);
//#endif
  
  //#ifdef DPC_PWM_xADVTIM_x CH Future dev.  
  //      .... Put here the code 
  //#endif             
}





/**
* @brief  DPC_PWM_OutDisable Output DISBLE PWM generator 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_OutDisable(void)
{
//#ifdef USE_ADVTIM
//    DPC_PWM_ADVTIM_OutDisable();                                                ///Safe: Disable ADVTIM outputs if enabled
//#elif USE_HRTIM
    DPC_PWM_HRTIM_OutDisable();                                                 ///Safe: Disable HRTIM outputs if enabled
//#else
//    SELECT DEFINE
//#endif
}





/**
* @brief  DPC_PWM_OutEnable Output DISBLE PWM generator 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_OutEnable(DPC_PWM_TypeDef *tDPC_PWM_loc)
{
  if(tDPC_PWM_loc->DPC_PWM_Status==PWM_Armed){    
//#ifdef USE_ADVTIM
//    DPC_PWM_ADVTIM_OutEnable();
//#elif USE_HRTIM
    DPC_PWM_HRTIM_OutEnable();
    HAL_GPIO_WritePin(PFC_SW_SRC_GPIO_Port, PFC_SW_SRC_Pin, GPIO_PIN_SET);
//#else
//    SELECT DEFINE
//#endif
  }
}






/**
* @brief  DPC_PWM_Start 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/

void DPC_PWM_Start(void)
{
//#ifdef USE_ADVTIM
//  DPC_PWM_ADVTIM_PWMStart();                                                        ///
//  DPC_PWM_ADVTIM_OutDisable();                                                  ///Safe: Disable ADVTIM outputs if enabled
//#elif USE_HRTIM
  DPC_PWM_HRTIM_Start();                                                        ///
  DPC_PWM_HRTIM_OutDisable();                                                   ///Safe: Disable HRTIM outputs if enabled
//#else
//  SELECT DEFINE
//#endif
}




/**
* @brief  DPC_PWM_Init 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_PWM_Init(uint32_t  BURST_PWM_Freq_Desidered,uint32_t  PWM_Freq_Desidered,DPC_PWM_StatusTypeDef DPC_PWM_SET, DPC_PWM_TypeDef *tDPC_PWM_loc, DMA_PWMDUTY_STRUCT *DUTY_SRC)
{

  uint32_t PWM_Period;                                                          ///
  uint32_t BURST_PWM_Period;                                                    ///  
  uint32_t Timers_Clock;                                                        ///
  uint32_t f_tim_ket_ck;                                                        ///  
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  
  
  Timers_Clock=HAL_RCC_GetPCLK2Freq();                                      ///
  f_tim_ket_ck=Timers_Clock;                                                /// Represent frequency Internal clock source (tim_ker_ck) expressed in Hz - see: pag-1063 RM0440 Rev1
  PWM_Period=((f_tim_ket_ck/PWM_Freq_Desidered) - 1)*16;                       ///  uint32_t PWM_Period;
  BURST_PWM_Period=((f_tim_ket_ck/BURST_PWM_Freq_Desidered) - 1);           ///  uint32_t PWM_Period;
  tDPC_PWM_loc->dutyMaxLim=tempDEF_dutyMaxLim;                                  /// Adapt to PRESCALER
  tDPC_PWM_loc->dutyMinLim=tempDEF_dutyMinLim;                                  /// Adapt to PRESCALER
  
  tDPC_PWM_loc->PWM_Period=PWM_Period;                                          ///
  tDPC_PWM_loc->BURST_PWM_Period=BURST_PWM_Period;                              ///  
  tDPC_PWM_loc->DPC_PWM_Status=DPC_PWM_SET;                                     ///

  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_RST;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT1|HRTIM_TIMFAULTENABLE_FAULT3;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  pTimerCfg.DMASrcAddress = (uint32_t)&DUTY_SRC->phA;
  pTimerCfg.DMADstAddress = (uint32_t)&(hhrtim1.Instance->sTimerxRegs[0].CMP1xR);
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMASrcAddress = (uint32_t)&DUTY_SRC->phB;
  pTimerCfg.DMADstAddress = (uint32_t)&(hhrtim1.Instance->sTimerxRegs[1].CMP1xR);
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMASrcAddress = (uint32_t)&DUTY_SRC->phC;
  pTimerCfg.DMADstAddress = (uint32_t)&(hhrtim1.Instance->sTimerxRegs[2].CMP1xR);
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }


  DPC_PWM_Start();                                                              ///
  DPC_PWM_OutDisable();                                                         ///Safe: Disable PWM outputs if enabled 
}



/**
* @brief  Aggiorna il Pulse che rappresentail duty dei TIM1 e TIM8 che vengono usati 
*         per le PWM del convertitore utilizzando delle LUT
*
* @param  
*
* @retval Null 
*
* @note Function valid for STM32G4xx family   
*/
void RefreshDuty(uint16_t LUTsinePOS[],uint16_t LUTsineNEG[],treephaseSTRUCT *LUT_3PH,uint32_t  PWM_PERIOD_COUNTER)
{
  // Routine che incrementa l'indice di lettura utilizzando  una rampa (  LUT_3PH->ramp)
  uint16_t last_sample=(uint16_t)(PWM_PERIOD_COUNTER*LUT_3PH->discretization_coefficient)-1;
  if(LUT_3PH->ramp<last_sample) 
  {
    LUT_3PH->ramp=LUT_3PH->ramp+(LUT_3PH->step);
  }
  else
  {
    LUT_3PH->ramp=0;
  }
  if(LUT_3PH->ramp>=last_sample)
  {    
    LUT_3PH->ramp=last_sample; 
  }  
  LUT_3PH->rampA=(uint16_t)LUT_3PH->ramp+0;
  LUT_3PH->rampB=(uint16_t)LUT_3PH->ramp+LUT_3PH->ramp_offset;
  LUT_3PH->rampC=(uint16_t)LUT_3PH->ramp+LUT_3PH->ramp_offset+LUT_3PH->ramp_offset;
  if(LUT_3PH->rampB>=last_sample)
  {
    LUT_3PH->rampB=LUT_3PH->rampB-last_sample;
  }
  if(LUT_3PH->rampC>=last_sample)
  {
    LUT_3PH->rampC=LUT_3PH->rampC-last_sample;
  }
  TIM1->CCR1 =  LUTsinePOS[LUT_3PH->rampA];
  TIM1->CCR2 =  LUTsinePOS[LUT_3PH->rampB];
  TIM1->CCR3 =  LUTsinePOS[LUT_3PH->rampC];
  TIM8->CCR1 =  LUTsineNEG[LUT_3PH->rampA];
  TIM8->CCR2 =  LUTsineNEG[LUT_3PH->rampB];
  TIM8->CCR3 =  LUTsineNEG[LUT_3PH->rampC];
}




/**
* @brief  
*
* @param  
*
* @retval Null 
*
* @note Function valid for STM32G4xx family   
*/
void DPC_PWM_Set_HRTIM(float VA,float VB,float VC)
{
//#ifdef   STDES_VIENNARECT
//  uint16_t ValuePhA;
//  uint16_t ValuePhB;
//  uint16_t ValuePhC;
//  uint16_t Period;
//  Period=__HAL_HRTIM_GETPERIOD(&hhrtim1, 0x0);
//  ValuePhA =(uint16_t)(VA*Period);
//  ValuePhB =(uint16_t)(VB*Period);
//  ValuePhC =(uint16_t)(VC*Period);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_1,ValuePhB);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x0,HRTIM_COMPAREUNIT_2,ValuePhC);
//  __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x1,HRTIM_COMPAREUNIT_1,ValuePhA);
//#elif   STDES_PFCBIDIR_REV2
////
//#else
////  SELECT DEFINE  // ERROR TO PREVENT NO APPLICATION DEFINITION
//#endif
}



/**
  * @brief  
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void Send_Duty_2LC_SPWM_LUT(uint16_t VA,uint16_t VB,uint16_t VC)
{
  TIM1->CCR1 =  (uint32_t)(VA);
  TIM1->CCR2 =  (uint32_t)(VB);
  TIM1->CCR3 =  (uint32_t)(VC);
}


/**
  * @brief  Send_Duty_2LC_SPWM_OPT: Modulator Module to use (STDES-PFCBIDIR) in 2LC mode
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void Send_Duty_2LC_SPWM_OPT(float VA,float VB,float VC,uint32_t  HALF_PWM_PERIOD_COUNTER_INT)
{
  TIM1->CCR1 =  (uint16_t)((VA+1)*HALF_PWM_PERIOD_COUNTER_INT);
  TIM1->CCR2 =  (uint16_t)((VB+1)*HALF_PWM_PERIOD_COUNTER_INT);
  TIM1->CCR3 =  (uint16_t)((VC+1)*HALF_PWM_PERIOD_COUNTER_INT); 
}




/**
  * @brief  
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void Send_Duty_2LC_SPWM(float VA,float VB,float VC,uint32_t  PWM_PERIOD_COUNTER_INT)
{
 
  TIM1->CCR1 =  (uint32_t)(((VA+1)*0.5)*(float)PWM_PERIOD_COUNTER_INT);
  TIM1->CCR2 =  (uint32_t)(((VB+1)*0.5)*(float)PWM_PERIOD_COUNTER_INT);
  TIM1->CCR3 =  (uint32_t)(((VC+1)*0.5)*(float)PWM_PERIOD_COUNTER_INT);
}




/**
  * @brief  Absolute value function used previous of Modulator in (STDES-VIENNARECT)
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void VIENNA_MOD_VCTRL(TRANSFORM_ABC_t *VIENNA_MOD_ABC_CTRL)
{

if(VIENNA_MOD_ABC_CTRL->axA>=0){
VIENNA_MOD_ABC_CTRL->axA=VIENNA_MOD_ABC_CTRL->axA;
}
else 
{
  VIENNA_MOD_ABC_CTRL->axA=VIENNA_MOD_ABC_CTRL->axA*-1.0;
}  
  
  
if(VIENNA_MOD_ABC_CTRL->axB>=0){
VIENNA_MOD_ABC_CTRL->axB=VIENNA_MOD_ABC_CTRL->axB;
}
else 
{
  VIENNA_MOD_ABC_CTRL->axB=VIENNA_MOD_ABC_CTRL->axB*-1.0;
}

if(VIENNA_MOD_ABC_CTRL->axC>=0){
VIENNA_MOD_ABC_CTRL->axC=VIENNA_MOD_ABC_CTRL->axC;
}
else 
{
  VIENNA_MOD_ABC_CTRL->axC=VIENNA_MOD_ABC_CTRL->axC*-1.0;
}

}



