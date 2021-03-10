/**
******************************************************************************
* @file    DPC_Application_Conf.h
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
#ifndef __DPC_APPLICATION_CONF_H
#define __DPC_APPLICATION_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define SQRT_2           1.41421                                                        /*!< Square Root of number 2 */ 
#define SQRT_3           1.73205                                                        /*!< Square Root of number 3 */ 

/* Exported macro ------------------------------------------------------------*/

//PWM Generation Timer configuration Section
// Tipology: - DPC_PWM_1ADVTIM_3CH, use 1 Advanced Control Timer with 3 channel.
//           - DPC_PWM_2ADVTIM_3CH_3CHX, use 1 Advanced Control Timer with 3 channel + complementary channels.

#define STDES_PFCBIDIR


///__Start_________________________________________________________________STDES_PFCBIDIR_REV2_______________________________________________________________________

///DPC Finite State Machine DEFINE of STDESPFCBIDIR_REV2

#define DPC_PC_State_Init               FSM_Idle                                        /*!< PC_State_TypeDef */
#define DPC_FSM_RUN_INIT                Run_Idle                                        /*!< Run_State_TypeDef */

//#define DPC_PC_State_Init               FSM_Debug                                     /*!< PC_State_TypeDef */
//#define DPC_FSM_RUN_INIT                Run_PFC_Mode                                  /*!< Run_State_TypeDef */

//#define DPC_CTRL_INIT                   OPEN_LOOP                                     /*!<  DPC Init Operation */
//#define DPC_CTRL_INIT                   CURRENT_LOOP                                  /*!<  DPC Init Operation */
#define DPC_CTRL_INIT                   VOLTAGE_LOOP                                    /*!<  DPC Init Operation */


#define DPC_PWM_INIT                    PWM_Armed                                       /*!<Use during normal operation*/
//#define DPC_PWM_INIT                    PWM_Safe                                      /*!<Use during Sensing debugging*/



///_________________________________________________________________________BASIC APPLICATION CONFIGURATOR______________________________________


///AC MAIN DEFINE of STDES-PFCBIDIR
#define DPC_VAC_220                                                                     /*!< Nominal AC Input Voltage - (Expressed in VOLT)*/
//#define DPC_VAC_110                                                                     /*!< Nominal AC Input Voltage - (Expressed in VOLT)*/
#define DPC_VAC                         180//180                                             /*!< Min AC Input Voltage - (Expressed in VOLT)*/
#define DPC_VDC                         300//180										/*!< Min DC Voltage to set relays closed*/
#define DPC_AC_3W                                                                       /*!< 3-WIRE AC Main Connection */
//#define DPC_AC_4W                                                                       /*!< 4-WIRE AC Main Connection */
///DC OUTPUT  DEFINE of STDES-PFCBIDIR
#define DPC_PFC_VDC_OUT                 700//720                                             /*!< DPC - DC Outout Voltage referance value of the Power converetr [Expresed in Volt]*/
///PROTECTION
#define DPC_VAC_RMS_OV                  400                                             /*!< Over Voltage Limit AC main RMS Value [Expressed in Volts]*/
#define DPC_VAC_RMS_UVLO                100//150                                             /*!< [Under Voltage Lock Out RMS Value [Expressed in Volts]*/
#define DPC_VAC_RMS_UV                  50//50                                              /*!< [Under Voltage RMS Value [Expressed in Volts]*/


///_________________________________________________________________________ADV APPLICATION CONFIGURATOR______________________________________

///Inrush DEFINE of STDES-PFCBIDIR
#if defined(DPC_AC_3W)
#define INRUSH_VREF_V                   (uint16_t)((float)DPC_VAC*SQRT_2*SQRT_3)        /*!< INRUSH DC Voltage threshold - (Expressed in VOLT)*/
#elif defined(DPC_AC_4W)
#define INRUSH_VREF_V                   (uint16_t)((float)DPC_VAC*SQRT_2*2.0)           /*!< INRUSH DC Voltage threshold - (Expressed in VOLT)*/
#endif
#define DPC_INRS_EN                     RESET                                             /*!<*/
#define INRUSH_VLIM                     30                                              /*!< [Expressed in volt]*/


///DPC START Burst Define of STDES-PFCBIDIR
#define DPC_STARTBURST_DUTY             0.05                                             /*!< [Expressed in Unit]*/
#define STARTBURST_VREF_V               DPC_PFC_VDC_OUT                                 /*!< [Expressed in Volts]*/
#define START_BURST_VHIST               5                                              /*!< [Expressed in Volts]*/
#define DPC_STARTBURST_EN               SET                                             /*!< [Expressed in Boolean]*/ 


///DPC Run Burst Define of STDES-PFCBIDIR
#ifdef DPC_VAC_220
#define DPC_BURST_DUTY_NL               0.15                                            /*!< [Expressed in Unit]*/
#define DPC_BURST_DUTY_LL               0.25                                            /*!< [Expressed in Unit]*/
#elif DPC_VAC_110
#define DPC_BURST_DUTY_NL               0.4                                             /*!< [Expressed in Unit]*/
#define DPC_BURST_DUTY_LL               0.7                                             /*!< [Expressed in Unit]*/
#endif
#define RUN_BURST_VREF_V                DPC_PFC_VDC_OUT                                 /*!< [Expressed in Volts]*/
#define RUN_BURST_VHIST                 10                                              /*!< [Expressed in Volts]*/
#define DPC_BURST_EN                    SET                                             /*!< [Expressed in Boolean]*/    


///DPC Theshold of STDES-PFCBIDIR
#define DPC_START_NO_LOAD_CURR          20                                             /*!< [Expressed in AMPs x 10]*/
#define DPC_START_LOW_LOAD_CURR         30                                              /*!< [Expressed in AMPs x 10]*/
#define DPC_NO_LOAD_CURR                10                                             /*!< [Expressed in AMPs x 10]*/
#define DPC_LOW_LOAD_CURR               15                                             /*!< [Expressed in AMPs x 10]*/
#define DPC_OVER_LOAD_CURR              20                                              /*!< [Expressed in AMPs]*/
#define DPC_NO_LOAD_DELTA_CURR          20                                              /*!< [Expressed in % Percentage]*/
#define DPC_LOW_LOAD_DELTA_CURR         30                                              /*!< [Expressed in % Percentage]*/
#define DPC_VAC_PK_OV                   (uint16_t)((float)DPC_VAC_RMS_OV*SQRT_2)        /*!< Over Voltage Limit AC main Peak Value  [Expressed in Volts]*/
#define DPC_VAC_PK_UVLO                 (uint16_t)((float)DPC_VAC_RMS_UVLO*SQRT_2)      /*!< [Under Voltage Lock Out Peak Value [Expressed in Volts]*/
#define DPC_VAC_PK_UV                   (uint16_t)((float)DPC_VAC_RMS_UV*SQRT_2)        /*!< [Under Voltage Peak Value [Expressed in Volts]*/
#define DPC_VAC_MIN                     20                                              /*!< [Expressed in Volts]*/
#define DPC_IAC_MAX                     30                                              /*!< [Expressed in Amps]*/




///DPC LOAD Define of STDESPFCBIDIR
#define DPC_VDC_OV                      750//850                                             /*!< DPC Output Voltage Higher Limit [Expressed in Volts]*/
//#define DPC_VDC_UV                      700                                             /*!< DPC Output Voltage Lower Limit [Expressed in Volts]*/
#define DPC_VDC_UV                      400                                             /*!< DPC Output Voltage Lower Limit [Expressed in Volts]*/
#define DPC_VCAP_LIM                    450                                             /*!< DPC Capacitor Voltage Limit [Expressed in Volts]*/


///_________________________________________________________________________CONTROL CONFIGURATOR______________________________________

///DPC Task DEFINE of STDESPFCBDIR
#define RefreshTime_DESIDERED           10000//20000                                           /*!< Expressed in hz */
#define RefreshTime_TO_DESIDERED        1000                                            /*!< Expressed in hz */ 
#define RefreshTime2_DESIDERED          5000//3000                                            /*!< Expressed in hz */
#define DPC_PI_ID_TS                    ((float)1/RefreshTime_DESIDERED)                /*!< Discrete time step of the PI regulator related to d-axis */
#define DPC_PI_IQ_TS                    ((float)1/RefreshTime_DESIDERED)                /*!< Discrete time step of the PI regulator related to q-axis */
#define DPC_PI_VDC_TS                   ((float)1/RefreshTime_DESIDERED)                /*!< Discrete time step of the PI regulator related to DC voltage control */
#define DPC_PLL_TS                      ((float)1/RefreshTime2_DESIDERED)               /*!< PLL - Task time step [expressed in sec]*/

///DPC VCTRL Define of STDESPFCBIDIR
#define DPC_VCTRL_KP                    4E-4                                            /*!< VCTRL - Proportional gain of the PI regulator related to DC voltage control*/
#define DPC_VCTRL_KI                    0.3                                             /*!< VCTRL - Integral gain of the PI regulator related to DC voltage control*/
#define DPC_PFC_VDC                     DPC_PFC_VDC_OUT                                 /*!< VCTRL - DC Voltage referance value of the PFC [Expresed in Volt]*/
#define DPC_PFC_Iref_sat                10//22                                              /*!< VCTRL - d-q axis AC Current referance limit of the PFC [Expresed in AMPs]*/
#define DPC_VCTRL_PI_AWTG               0.02                                            /*!< VCTRL - Anti Wind-up GAIN*/
#define DPC_VCTRL_PI_sat_up             ((float)DPC_PFC_Iref_sat/(float)G_IAC)          /*!< VCTRL - Higher Current Referance Saturation LIMIT*/
#define DPC_VCTRL_PI_sat_down           0                                               /*!< VCTRL - Lower Current Referance Saturation LIMIT*/
#define DPC_VCTRL_PI_SAT_EN             SET                                             /*!< VCTRL - Current Referance Saturation Enable*/
#define DPC_VCTRL_PI_AW_EN              SET                                             /*!< VCTRL - Anti Wind-up Enable*/


///DPC CDC Define of STDESPFCBIDIR
#if defined(DPC_AC_3W)                  ///DPC CDC Define of STDESPFCBIDIR for 3-Wire connection
#define DPC_ID_KP                       1.706e-01                                       /*!< CDC - Proportional gain of the PI regulator related to d-axis*/
#define DPC_ID_KI                       5.794e+02                                       /*!< CDC - Integral gain of the PI regulator related to d-axis*/
#define DPC_IQ_KP                       DPC_ID_KP                                       /*!< CDC - Proportional gain of the PI regulator related to q-axis*/
#define DPC_IQ_KI                       DPC_ID_KI                                       /*!< CDC - Integral gain of the PI regulator related to q-axis*/
#define DPC_PI_ID_AWTG                  0.01                                            /*!< CDC - d-axis Anti Wind-up GAIN*/
#define DPC_PI_IQ_AWTG                  DPC_PI_ID_AWTG                                  /*!< CDC - q-axis Anti Wind-up GAIN*/
#define DPC_PI_ID_sat_up                0.9                                             /*!< CDC - Higher d-axis Duty Referance Saturation LIMIT*/
#define DPC_PI_ID_sat_down              -0.9                                            /*!< CDC - Lower d-axis Duty Referance Saturation LIMIT*/
#define DPC_PI_IQ_sat_up                0.1                                             /*!< CDC - Higher q-axis Duty Referance Saturation LIMIT*/
#define DPC_PI_IQ_sat_down              -0.1                                            /*!< CDC - Lower q-axis Duty Referance Saturation LIMIT*/
#elif defined(DPC_AC_4W)                ///DPC CDC Define of STDESPFCBIDIR for 4-Wire connection
#define DPC_ID_KP                       1.706e-01                                       /*!< CDC - Proportional gain of the PI regulator related to d-axis*/
#define DPC_ID_KI                       5.794e+02                                       /*!< CDC - Integral gain of the PI regulator related to d-axis*/
#define DPC_IQ_KP                       DPC_ID_KP                                       /*!< CDC - Proportional gain of the PI regulator related to q-axis*/
#define DPC_IQ_KI                       DPC_ID_KI                                       /*!< CDC - Integral gain of the PI regulator related to q-axis*/
#define DPC_PI_ID_AWTG                  0.01                                            /*!< CDC - d-axis Anti Wind-up GAIN*/
#define DPC_PI_IQ_AWTG                  DPC_PI_ID_AWTG                                  /*!< CDC - q-axis Anti Wind-up GAIN*/
#define DPC_PI_ID_sat_up                0.9                                             /*!< CDC - Higher d-axis Duty Referance Saturation LIMIT*/
#define DPC_PI_ID_sat_down              -0.9                                            /*!< CDC - Lower d-axis Duty Referance Saturation LIMIT*/
#define DPC_PI_IQ_sat_up                0.1                                             /*!< CDC - Higher q-axis Duty Referance Saturation LIMIT*/
#define DPC_PI_IQ_sat_down              -0.1                                            /*!< CDC - Lower q-axis Duty Referance Saturation LIMIT*/
#endif

#define DPC_PI_ID_SAT_EN                SET                                             /*!< CDC - d-axis Duty Referance Saturation Enable*/
#define DPC_PI_ID_AW_EN                 SET                                             /*!< CDC - d-axis Anti Wind-up Enable*/
#define DPC_PI_IQ_SAT_EN                DPC_PI_ID_SAT_EN                                /*!< CDC - q-axis Duty Referance Saturation Enable*/
#define DPC_PI_IQ_AW_EN                 DPC_PI_ID_AW_EN                                 /*!< CDC - q-axis Anti Wind-up Enable*/
#define CDC_VDC_FF_INIT                 RESET                                           /*!< CDC - DC Feef Forward Enable*/  
#define CDC_FF_Init                     RESET                                           /*!< CDC - AC Feef Forward Enable*/  
#define CDC_DEC_INIT                    RESET                                           /*!< CDC - Decoupling Enable*/  


///PLL DEFINE of STDES-PFCBIDIR
#define PLL_KP                          20                                              /*!< PLL - Proportional gain of Phase Loched Loop algorithm*/
#define PLL_KI                          500//5000                                            /*!< PLL - Integral gain of Phase Loched Loop algorithm*/
#define PLL_PHI_2pi                     -1.570796                                       /*!< PLL - Constant "-pi/2" [Expressed in rad]*/
#define PLL_DELTA_F                     20                                              /*!<*/
#define PLL_FF_Hz                       50                                              /*!< PLL - Feed Forward term of the VTO in Phase Loched Loop algorithm [Expressed in Hz]*/
#define DPC_PLL_SAT_EN                  SET                                             /*!< PLL - Saturation Enable*/ 
#define DPC_PLL_FGRID                   50                                              /*!< [Expressed in Hz]*/
#define DPC_PLL_OMEGAGRID               314                                             /*!< [Expressed in rad]*/
#define DPC_INDUCTOR                    900e-6//470e-6                                          /*!< [Expressed in Henry]*/
#define DPC_PLL_PIsat_up                50                                              /*!< [Expressed in Hz]*/
#define DPC_PLL_PIsat_down              -50                                             /*!< [Expressed in Hz]*/


///_________________________________________________________________________PERIPHERALS CONFIGURATOR______________________________________


///DPC LED Define of STDSES-PFCBIDIR
#define DPC_BLED_TIM                    htim15                                          /*!<*/
#define DPC_BLED_CH                     TIM_CHANNEL_1                                   /*!<*/


///DPC DAC Define of STDESPFCBIDIR
#define DAC_CH1_INIT                    12                                              /*!PLL theta_out = 12<*/
#define DAC_CH2_INIT                    3                                               /*!V_ABC_CTRL_A = 0 V_ABC_CTRL_B = 1 V_ABC_CTRL_C = 2 CDC.Id_feed = 3 CDC.Id_ref 4<*/
#define DAC_CH3_INIT                    4                                               /*!V_ABC_CTRL_A = 0 V_ABC_CTRL_B = 1 V_ABC_CTRL_C = 2 CDC.Id_feed = 3 CDC.Id_ref 4<*/
#define DAC_G_CH1_Init                  2048                                            /*!<*/
#define DAC_G_CH2_Init                  2048                                            /*!<*/
#define DAC_G_CH3_Init                  2048                                            /*!<*/
#define DAC_B_CH1_Init                  2048                                            /*!<*/
#define DAC_B_CH2_Init                  2048                                            /*!<*/
#define DAC_B_CH3_Init                  2048                                            /*!<*/



///ADC Gain STDES-PFCBIDIR 
#define G_VAC                           4.25//4.708                                           /*!< Gain terms of the AC voltage sensing */
#define B_VAC                           1975                                            /*!< Bias terms of the AC voltage sensing */
#define G_IAC                           32.5//42.67                                           /*!< Gain terms of the AC current sensing */
#define B_IAC                           1958                                          /*!< Bias terms of the AC current sensing */
#define G_VDC                           7.87//7.726                                           /*!< Gain terms of the DC voltage sensing */
#define B_VDC                           0                                               /*!< Bias terms of the DC voltage sensing */
#define G_IDC                           102.4                                           /*!< Gain terms of the DC current sensing */
#define B_IDC                           2048                                            /*!< Bias terms of the DC current sensing */


///DPC TIMERs Define of STDESPFCBIDIR
#define PWM_FREQ                        40000                                           /*!< Switching Frequency of converter expressed in [Hz]*/
#define DPC_DT                          0                                               /*!< PWM - Dead Time [Expressed in sec]*/
#define DPC_BURST_PWM_FREQ              20000                                           /*!< Switching Frequency of converter expressed in [Hz]*/
#define tempDEF_dutyMaxLim              (33999)
#define tempDEF_dutyMinLim              (0)


///DPC TIMEOUTs Define of STDESPFCBIDIR
#define TO_IDLE_Tick                    1000                                           /*!< Timeout Index - TimeOut_IDLE [mills]*/

//Relays delay
#define RELAY_TO_CH						1
#define RELAY_TIMEOUT 					5000
#define FSM_START_TO_CH					1
#define FSM_START_TIMEOUT 				5000

/* Exported functions ------------------------------------------------------- */

#endif //__DPC_APPLICATION_CONF_H
