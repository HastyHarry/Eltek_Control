/**
******************************************************************************
* @file    DPC_Lib_Conf.h
* @brief   This file contains the DPC Library Configuration.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPC_LIB_CONF_H
#define __DPC_LIB_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal_conf.h"
#include "DPC_Application_Conf.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

//PWM Generation Timer configuration Section
// Tipology: - DPC_PWM_1ADVTIM_3CH, use 1 Advanced Control Timer with 3 channel.
//           - DPC_PWM_2ADVTIM_3CH_3CHX, use 1 Advanced Control Timer with 3 channel + complementary channels.


#define STDES_PFCBIDIR_REV2
///__Start_________________________________________________________________STDES_PFCBIDIR_______________________________________________________________________
#ifdef STDES_PFCBIDIR_REV2

//#ifdef USE_ADVTIM                                               //ifdef
//#define DPC_PWM_2ADVTIM_3CH_3CHX                                                ///Use 2 Adv. Tim. 3 channel + 3 complementary channel
//#define PWM_Tim1                        htim1                                   /*!<PWM TIMER - PWM group 1*/
//#define PWM_Tim2                        htim8                                   /*!<PWM TIMER - PWM group 2*/
//#define PWM_CHANNEL_1                   TIM_CHANNEL_1                           /*!<PWM Channel Phase C (Power Board)*/
//#define PWM_CHANNEL_2                   TIM_CHANNEL_2                           /*!<PWM Channel Phase B (Power Board)*/
//#define PWM_CHANNEL_3                   TIM_CHANNEL_3                           /*!<PWM Channel Phase A (Power Board)*/
//#elif USE_HRTIM                                                 //ifdef
#define DPC_PWM_1HRTIM_6CH_6CHX                                                ///Use 2 Adv. Tim. 3 channel + 3 complementary channel
#define PWM_Tim1                        hhrtim1                                 /*!<PWM TIMER - PWM group 1*/
#define PWM_Tim2                        hhrtim1                                 /*!<PWM TIMER - PWM group 2*/
#define PWM_CHANNEL_1                   HRTIM_OUTPUT_TA1                        /*!<PWM Channel Phase x (Power Board)*/
#define PWM_CHANNEL_2                   HRTIM_OUTPUT_TB1//HRTIM_OUTPUT_TA2                        /*!<PWM Channel Phase x (Power Board)*/
#define PWM_CHANNEL_3                   HRTIM_OUTPUT_TC1//HRTIM_OUTPUT_TB2                        /*!<PWM Channel Phase x (Power Board)*/
//#endif                                                          //andif


#define APPL_Tim1                       htim2                                   /*!< Application TIMER - High Frequency Tasks*/
#define APPL_Tim2                       htim3                                   /*!< Application TIMER - Low Frequency Tasks*/
#define APPL_Tim3                       htim6                                   /*!< Application TIMER - Very Low Frequency Tasks*/
#define APPL_Tim4                       htim15                                  /*!< Application TIMER - BICOLOR LED managment*/


//Relays command Section
// Tipology: - RELAY_GRID_x, Relays used to connect ot the GRID. max 4 relays.
//           - RELAY_SER_1, Relays used to other services. max 3 ralays
//           - RELAY_FAN, Relay used for the FAN.       
#define USE_RELAY_GRID_A                                                        /*!<*/

//26.01.2021
//#define RELAY_GRID_A_PORT               ZVD_C__RLY_A_GPIO_Port                  /*!<*/
//#define RELAY_GRID_A_PIN                ZVD_C__RLY_A_Pin                        /*!<*/

#define USE_RELAY_GRID_B
//26.01.2021/*!<*/
//#define RELAY_GRID_B_PORT               ZVD_B__RLY_B_GPIO_Port                  /*!<*/
//#define RELAY_GRID_B_PIN                ZVD_B__RLY_B_Pin                        /*!<*/

#define USE_RELAY_GRID_C                                                        /*!<*/
//26.01.2021
//#define RELAY_GRID_C_PORT               ZVD_A__RLY_C_GPIO_Port                  /*!<*/
//#define RELAY_GRID_C_PIN                ZVD_A__RLY_C_Pin                        /*!<*/

//#define USE_RELAY_GRID_N
//#define RELAY_GRID_N_PORT             GPIOB 
//#define RELAY_GRID_N_PIN              GPIO_PIN_11    

#define USE_RELAY_SER_1                                                         /*!<*/
//26.01.2021
//#define RELAY_SER_1_PORT                RELAY_GPIO_Port                         /*!<*/
//#define RELAY_SER_1_PIN                 RELAY_Pin                               /*!<*/

//#define USE_RELAY_SER_2
//#define RELAY_SER_2_PORT              GPIOB 
//#define RELAY_SER_2_PIN               GPIO_PIN_5
//
//#define USE_RELAY_SER_3
//#define RELAY_SER_3_PORT              GPIOB 
//#define RELAY_SER_3_PIN               GPIO_PIN_6

//#define USE_RELAY_SER_4
//#define RELAY_SER_4_PORT              GPIOx 
//#define RELAY_SER_4_PIN               GPIO_PIN_n

#define USE_RELAY_FAN                                                           /*!<*/
//26.01.2021
//#define RELAY_FAN_PORT                  FAN_GPIO_Port                           /*!<*/
//#define RELAY_FAN_PIN                   FAN_Pin                                 /*!<*/

///DPC TIMEOUTs Define of STDESPFCBIDIR
#define TO_INRUSH                       TO_INDX1                                /*!< */
#define TO_OTHER                        TO_INDX2                                /*!< */
#define TO_IDLE                         TO_INDX3                                /*!< Timeout Index - TimeOut_IDLE [IDX]*/
#define TO_MISC2                        TO_INDX4                                /*!< */

///DPC ADCs Define of STDESPFCBIDIR
#define ADC1_CHs                        8                                       /*!< */
#define ADC2_CHs                        3                                       /*!< */

///__End_________________________________________________________________STDES_PFCBIDIR_______________________________________________________________________


#else
SELECT DEFINE
#endif    





/* Exported functions ------------------------------------------------------- */

#endif //__DPC_LIB_CONF_H
