/**
  ******************************************************************************
  * @file           : DPC_PLL.c
  *
  * @brief  La libreria DPC_PLL.h integra un algoritmo di "PHASE LOCKED LOOP" basato su VTO 
  *         che è applicato al sistema stazionario ottenuto mediante trasformate 
  *         Il risultato ottenuto è la frequenza della terna in ingresso e la relativa fase
  *
   * @note   La struttura dell'algoritmo PLL richiede un flusso di operazioni:
  *             -Ingresso delle tensioni trifase normalizzati (-1,1) - Optional Sharing "DATA.h"
  *             -Una trasformazione attraverso la ClarkePark di "Transform.h"
  *             -Utilizzo di un PI di "PI_LIB.h" per ottenere la frequenza 
  *             -Funzione di integrazione di "Integrators.h" per ottenere la fase
  *             -Output di Frequenza e Fase - Optional Sharing "DATA.h"
  * @dipendence "DPC_Transforms.h" "DPC_Pid.h" "Integrator.h" "DPC_Datacollector.h"
  *
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


/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

#include "DPC_Pid.h"
#include "DPC_PLL.h"
#include "main.h"
#include "DPC_Transforms.h"
#include "DPC_Faulterror.h"
#include "DPC_FSM.h"

//#include "Intregrators.h" DA IMPORTARE DA STNRGprj

/* Private variables ---------------------------------------------------------*/

float theta_ref_int_pll;
PI_STRUCT_t PI_PLL;
INTEGRATOR_STRUCT INTEGRATOR_PLL;
TRANSFORM_QDO_t ClarkeParkPLL;

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  DPC_PLL_Init: PLL Init function.
  * @param  PLL_sub:
  * @param  kp_pll:
  * @param  ki_pll:
  * @param  Ts_pll:
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void DPC_PLL_Init(PLL_Struct *PLL_sub, float kp_pll, float ki_pll, float Ts_pll, float phi_2pi, float delta_freq, uint8_t uFeedforward_Hz, FlagStatus satPI_toggle,float PIsat_up,float PIsat_down){

PLL_sub->PLL_Enable=SET;
PLL_sub->kp_pll=kp_pll;
PLL_sub->ki_pll=ki_pll;
PLL_sub->k0_pll=kp_pll;
PLL_sub->k1_pll=ki_pll*Ts_pll;
PLL_sub->Ts_pll=Ts_pll;
PLL_sub->pll_phi_2pi=phi_2pi;
PLL_sub->delta_freq=delta_freq;
PLL_sub->uFreqFeedforwardHz=uFeedforward_Hz;
PLL_sub->pi_pll.satPI_toggle=satPI_toggle;
PLL_sub->pi_pll.PIsat_up=PIsat_up;
PLL_sub->pi_pll.PIsat_down=PIsat_down;
if(PLL_sub->PLL_Enable==SET){
PLL_sub->Status_PLL=PLL_IDLE;
}
else if(PLL_sub->PLL_Enable==RESET){
PLL_sub->Status_PLL=PLL_DISABLED;
}  
}

 
/**
  * @brief  PLLabc: PLL abc function.
  * @param  PLL_sub: pointer to Pll struct.
  * @param  VAC_PLL: pointer to VAC Pll struct
  * @param  Theta_out: pointer to theta val output variable.
  * @param  omega_pi_out: pointer to omega pi val output variable.
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
void PLLabc(PLL_Struct *PLL_sub, VoltageAC_PLL_Struct *VAC_PLL,float *theta_out,  float *omega_pi_out){ 
float pll_va_sub=VAC_PLL->VphA;
float pll_vb_sub=VAC_PLL->VphB;
float pll_vc_sub=VAC_PLL->VphC;
float kp_pll_sub=PLL_sub->kp_pll;
float ki_pll_sub=PLL_sub->ki_pll;
float pll_theta_in_sub = PLL_sub->pll_theta_in;
float pll_theta_out_sub = PLL_sub->pll_theta_out;
float pll_d_sub;
float pll_q_sub;
float pll_o_sub;
float Ts_pll_sub = PLL_sub->Ts_pll;
float omega_pi_pll;
float omega_pi_ff_pll_sub;

   
pll_theta_in_sub=pll_theta_out_sub;
PLL_sub->pll_theta_in=pll_theta_in_sub;

  // Tranform
  Clarke_Park(pll_va_sub, pll_vb_sub, pll_vc_sub, pll_theta_in_sub,PLL_sub->pll_phi_2pi,&pll_d_sub, &pll_q_sub, &pll_o_sub);
 
PLL_sub->pll_d=pll_d_sub;
PLL_sub->pll_q=pll_q_sub;
PLL_sub->pll_o=pll_o_sub;
     
// LOOP FILTER PI
                  
PI_PLL.k0=kp_pll_sub; //K0=Kp
PI_PLL.k1=ki_pll_sub*Ts_pll_sub; //K1=Ki*Ts
PI_PLL.satPI_toggle=RESET;

          
PI(0, pll_q_sub , &PI_PLL);

PLL_sub->pi_pll=PI_PLL;          
              
           
       omega_pi_pll=PI_PLL.PIout;

// FEEDFORWARD
      omega_pi_ff_pll_sub=omega_pi_pll+50;

//  VTO - SATURATED INTEGRATOR   
      

       INTEGRATOR_PLL.Ts=Ts_pll_sub; 
    
      Integral(&INTEGRATOR_PLL,omega_pi_ff_pll_sub);
   
     if (INTEGRATOR_PLL.Integralout>1||INTEGRATOR_PLL.Integralout<-1)
     {
              INTEGRATOR_PLL.Integralout=0;
     }
     else
     {
       INTEGRATOR_PLL.Integralout=INTEGRATOR_PLL.Integralout;
     }
 
 PLL_sub->integrator_pll=INTEGRATOR_PLL;    
       
 pll_theta_out_sub=INTEGRATOR_PLL.Integralout;

    *theta_out=pll_theta_out_sub;
    *omega_pi_out=omega_pi_ff_pll_sub;
//    PLL_sub->pll_theta_out=pll_theta_out_sub;
    PLL_sub->pll_theta_out=pll_theta_out_sub;

    PLL_sub->pll_theta_out_2pi= PLL_sub->pll_theta_out*DOUBLE_PI;

    PLL_sub->omega_ff_pll=omega_pi_ff_pll_sub;
}


/**
  * @brief  PLLabc_opt: abc opt function.
  * @param  PLL_sub, pointer to Pll struct.
  * @param  VAC_PLL: pointer to VAC Pll struct
  * @param  Theta_out: pointer to theta val output variable.
  * @param  omega_pi_out: pointer to omega pi val output variable.
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void PLLabc_opt(PLL_Struct *PLL_sub, VoltageAC_PLL_Struct *VAC_PLL,float *theta_out,  float *omega_pi_out){

float pll_va_sub=VAC_PLL->VphA;
float pll_vb_sub=VAC_PLL->VphB;
float pll_vc_sub=VAC_PLL->VphC;
float kp_pll_sub=PLL_sub->kp_pll;
//float ki_pll_sub=PLL_sub->ki_pll;
float pll_theta_in_sub = PLL_sub->pll_theta_in;
float pll_theta_out_sub = PLL_sub->pll_theta_out;
float pll_d_sub;
float pll_q_sub;
float pll_o_sub;
float Ts_pll_sub = PLL_sub->Ts_pll;
float omega_pi_pll;
//float old_value_integrale2_pll;
//uint16_t theta_pll_int;
//float theta_pll_int_mod;
//float theta_pll_101;
float omega_pi_ff_pll_sub;

   
pll_theta_in_sub=pll_theta_out_sub;
PLL_sub->pll_theta_in=pll_theta_in_sub;

  // Trasformata
  Clarke_Park_Opt(pll_va_sub, pll_vb_sub, pll_vc_sub, pll_theta_in_sub,&pll_d_sub, &pll_q_sub, &pll_o_sub);

PLL_sub->pll_d=pll_d_sub;
PLL_sub->pll_q=pll_q_sub;
PLL_sub->pll_o=pll_o_sub;
     
// LOOP FILTER PI

PI_PLL.k0=kp_pll_sub; //K0=Kp
PI_PLL.k1=kp_pll_sub*Ts_pll_sub; //K1=Ki*Ts
PI_PLL.satPI_toggle=RESET;

PI(0, pll_q_sub , &PI_PLL);

PLL_sub->pi_pll=PI_PLL;          
              
           
      omega_pi_pll=PI_PLL.PIout;

// FEEDFORWARD
      omega_pi_ff_pll_sub=omega_pi_pll+50;

      //  VTO - SATURATED INTEGRATOR    //DA sostiruire con Integrator() di "integrator.h" 
      
      INTEGRATOR_PLL.Ts=Ts_pll_sub; 
      
      Integral(&INTEGRATOR_PLL,omega_pi_ff_pll_sub);
      
     if (INTEGRATOR_PLL.Integralout>1||INTEGRATOR_PLL.Integralout<-1)
     {
              INTEGRATOR_PLL.Integralout=0;
     }
     else
     {
       INTEGRATOR_PLL.Integralout=INTEGRATOR_PLL.Integralout;
     }
      
 PLL_sub->integrator_pll=INTEGRATOR_PLL;    
    
    pll_theta_out_sub=INTEGRATOR_PLL.Integralout;

    *theta_out=pll_theta_out_sub;
    *omega_pi_out=omega_pi_ff_pll_sub;
    PLL_sub->pll_theta_out=pll_theta_out_sub;
    PLL_sub->pll_theta_out_2pi= PLL_sub->pll_theta_out*DOUBLE_PI;
    PLL_sub->omega_ff_pll=omega_pi_ff_pll_sub;    
}


/**
  * @brief  DPC_PLL_pllqd_Run: PLL qd function.
  * @param  PLL_sub: pointer to Pll struct.
  * @param  VAC_PLL: pointer to VAC Pll struct
  * @param  Theta_out: pointer to theta val output variable [Normalized 0-1].
  * @param  omega_pi_out: pointer to omega pi val output variable.
  * 
  * @retval STATUS_PLL_TypeDef 
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
STATUS_PLL_TypeDef DPC_PLL_pllqd_Run(PLL_Struct *PLL_sub, VoltageAC_qd_PLL_Struct *VAC_qd_PLL,float *theta_out,  float *omega_pi_out){
 
  STATUS_PLL_TypeDef STATUS_PLL;
  
  if(PLL_sub->PLL_Enable)
  {
  
//Pass "PLL_sub" data
//float kp_pll_sub=PLL_sub->kp_pll;                                       ///Local variable to pass kp data
//float ki_pll_sub=PLL_sub->ki_pll;                                       ///Local variable to pass ki data
float k0_pll_sub=PLL_sub->k0_pll;                                       ///Local variable to pass k0 data
float k1_pll_sub=PLL_sub->k1_pll;                                       ///Local variable to pass k1 data
float pll_theta_in_sub = PLL_sub->pll_theta_in;                         /// Loacal variable to pass PLL theta input
float pll_theta_out_sub = PLL_sub->pll_theta_out;                       ///Local variable to pass PLL theta output
float Ts_pll_sub = PLL_sub->Ts_pll;                                     ///Local variable to pass Ts 
FlagStatus satPI_toggle_sub = PLL_sub->pi_pll.satPI_toggle;
float PIsat_down_sub=PLL_sub->pi_pll.PIsat_down;
float PIsat_up_sub=PLL_sub->pi_pll.PIsat_up;
//block!!

//Pass "VAC_qd_PLL" data 
float pll_d_sub=VAC_qd_PLL->Vph_d;      //Local variable to pass Vd
float pll_q_sub=VAC_qd_PLL->Vph_q;      //Local variable to pass Vq
float pll_o_sub=VAC_qd_PLL->Vph_o;      //Local variable to pass Vo

//Local Data 
float omega_pi_pll_sub;  //omega_pi_pll_sub represent the output of the PI of PLL
float omega_pi_ff_pll_sub; //omega_pi_pll_sub will be added by feedforward terms
  
  // Set in actual angle (input) the previous value (output) 
  pll_theta_in_sub=pll_theta_out_sub;
  //Save this new data in the "PLL_sub" struct
  PLL_sub->pll_theta_in=pll_theta_in_sub;
  
  //Pass and save the input "Vdq0" in "PLL_sub" struct
  PLL_sub->pll_d=pll_d_sub;
  PLL_sub->pll_q=pll_q_sub;
  PLL_sub->pll_o=pll_o_sub;
  
  //Configure the PI for PLL (k0, k1, SAT)
  // LOOP FILTER PI         
  PI_PLL.k0=k0_pll_sub; //K0=Kp
  PI_PLL.k1=k1_pll_sub; //K1=Ki*Ts
  
  PI_PLL.satPI_toggle=satPI_toggle_sub;
  PI_PLL.PIsat_up=PIsat_up_sub;
  PI_PLL.PIsat_down=PIsat_down_sub;
  
  
  //Task PI    
  PI(0, -pll_q_sub , &PI_PLL);
  
  //Pass the internal PI (PI_PLL) data in the "PLL_sub"
  PLL_sub->pi_pll=PI_PLL;          
  
  omega_pi_pll_sub=PI_PLL.PIout_sat;
  
  PLL_sub->omega_piout=omega_pi_pll_sub;
  
  // FEEDFORWARD
  omega_pi_ff_pll_sub=omega_pi_pll_sub+PLL_sub->uFreqFeedforwardHz;
  
  //  VTO - SATURATED INTEGRATOR    //Substitude with Integrator() of "integrator.h" 
  
  INTEGRATOR_PLL.Ts=Ts_pll_sub;  //Pass Ts(LocalVariable) to Integrator_PLL(struct)
  Integral(&INTEGRATOR_PLL,omega_pi_ff_pll_sub);   //Task Integrator    
  
  // SATURATE INTEGRATOR
  if (INTEGRATOR_PLL.Integralout>1)
  {
    INTEGRATOR_PLL.Integralout=0;
  }
  else if (INTEGRATOR_PLL.Integralout<0)
  {
    INTEGRATOR_PLL.Integralout=1;
  }
  else 
  {
    INTEGRATOR_PLL.Integralout=INTEGRATOR_PLL.Integralout;
  }
  
  //Pass INTEGRATOR_PLL(struct) to local PLL_sub(struct)
  PLL_sub->integrator_pll=INTEGRATOR_PLL;    
  
  pll_theta_out_sub=INTEGRATOR_PLL.Integralout;
  
  *theta_out=pll_theta_out_sub;
  *omega_pi_out=omega_pi_ff_pll_sub;
  
  PLL_sub->pll_theta_out=pll_theta_out_sub;
  
  PLL_sub->pll_theta_out_2pi= PLL_sub->pll_theta_out*DOUBLE_PI; //Scale normalized theta to (0-2pi)
  
  PLL_sub->omega_ff_pll=omega_pi_ff_pll_sub;
  
if((PLL_sub->omega_ff_pll-PLL_sub->uFreqFeedforwardHz)<PLL_sub->delta_freq&&(PLL_sub->omega_ff_pll-PLL_sub->uFreqFeedforwardHz)>-PLL_sub->delta_freq) ///CHECK
{
  PLL_sub->Status_PLL=PLL_SYNC;
  STATUS_PLL=PLL_SYNC;
}
else
{
  PLL_sub->Status_PLL=PLL_OUTRANGE;
  STATUS_PLL=PLL_OUTRANGE;
  DPC_FLT_Faulterror_Set(ERROR_PLL_OR);
}

  }
  else{
    PLL_sub->omega_ff_pll=0;
    PLL_sub->pll_theta_out=0;
    PLL_sub->pll_theta_out_2pi=0;
    STATUS_PLL=PLL_DISABLED;
  }
return STATUS_PLL;
}
  





