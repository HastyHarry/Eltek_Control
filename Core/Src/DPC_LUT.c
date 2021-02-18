/**
******************************************************************************
* @file           : DPC_LUT.c
* @brief          : LUT Module
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
#include "DPC_LUT.h"
#include "DPC_Math.h"
#include "math.h"
/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  DPC_LUT_Gen: Genera le LUT sinusoidali e di fase per il convertitore 
  *         
  *         
  * @param 
  *
  * @retval Null 
  *
  * @note   La generazione viene fatta customizzata per il numero di elementi e il settaggio dei timer
  *         Utilizza le funzioni matematiche sin di math.h 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void DPC_LUT_Gen(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *LUT_3PH_sub,uint32_t  PWM_PERIOD_COUNTER)
{  
  uint32_t i;                             ///Local index used in For Loop

  for(i=0;i<PWM_PERIOD_COUNTER;i++)             /// The for loop run from 0 to ARR Timer
  {    
   uint16_t stepLUT;   /*!< Used to determinate the effective step LUT caused by the LUT_SAMPLE/PWM_PERIOD_COUNTER (see discetization_coefficient) for details*/
    
    stepLUT=(uint16_t)((float)i*LUT_3PH_sub->discretization_coefficient);
    
    LUT_CONVERTER_sub->LUTramp[stepLUT]=i;          //Memorizza una rampa che va da 0 a stepLUT in un array di (n) elementi
    LUT_CONVERTER_sub->LUTramp_2pi[stepLUT]=LUT_CONVERTER_sub->LUTramp[stepLUT]*(1.0/(float)PWM_PERIOD_COUNTER)*6.28318531;             //Si normalizza la rampa (0-stepLUT) in (0-2pi)  
    LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]=LUT_3PH_sub->ma*sin((float)(LUT_CONVERTER_sub->LUTramp[stepLUT]/(float)PWM_PERIOD_COUNTER)*(float)6.283185307179586476925286766559);              // Memorizza il seno (-1,1)*ma
    LUT_CONVERTER_sub->LUTsineNEGfloat[stepLUT]=LUT_3PH_sub->ma*sin((float)(LUT_CONVERTER_sub->LUTramp[stepLUT]/(float)PWM_PERIOD_COUNTER)*(float)6.283185307179586476925286766559);              // Memorizza il seno (-1,1)*ma
    LUT_CONVERTER_sub->LUTsinefloat[stepLUT]=LUT_3PH_sub->ma*sin((float)(LUT_CONVERTER_sub->LUTramp[stepLUT]/(float)PWM_PERIOD_COUNTER)*(float)6.283185307179586476925286766559);              // Memorizza il seno (-1,1)*ma
    
#define STDES_PFCBIDIR_REV2
#if defined(STDES_PFCBIDIR) || defined(STDES_VIENNARECT) || defined(STDES_PFCBIDIR_REV2)

    if(i<((float)PWM_PERIOD_COUNTER*0.5))
    {  
      LUT_CONVERTER_sub->LUTsinePOS[stepLUT]= (uint16_t)(LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]*PWM_PERIOD_COUNTER);
      LUT_CONVERTER_sub->LUTsineNEG[stepLUT]=(uint16_t)(PWM_PERIOD_COUNTER+1); 
    }
    else
    {  
      LUT_CONVERTER_sub->LUTsinePOS[stepLUT]=0;
      LUT_CONVERTER_sub->LUTsineNEG[stepLUT]= (uint16_t)((LUT_CONVERTER_sub->LUTsineNEGfloat[stepLUT]+1.0)*PWM_PERIOD_COUNTER);
    }
#elif STDES_PFCBIDIR_2L  
    LUT_CONVERTER_sub->LUTsine[stepLUT]=(uint16_t)((LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]+1.0)*0.5*PWM_PERIOD_COUNTER);
    LUT_CONVERTER_sub->LUTsine01[stepLUT]=(float)((LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]+1.0)*0.5);
#else

    SELECT DEFINE
#endif
    
  } //for 
  LUT_3PH_sub->ramp_offset=(uint16_t)((float)PWM_PERIOD_COUNTER*0.33333333f*LUT_3PH_sub->discretization_coefficient);            // Calcola l'offset di lettura che rappresenta lo sfasamento di 120 gradi da usare nella routine di lettura
}




/**
  * @brief  DPC_LUT_Gen_opt: Genera le LUT sinusoidali e di fase per il convertitore 
  *         
  * @param 
  *
  * @retval Null 
  *
  * @note   La generazione viene fatta customizzata per il numero di elementi e il settaggio dei timer
  *         Utilizza le funzioni matematiche sin di math.h 
  *         
  * @note Function valid for STM32G4xx microconroller family   
  */
void DPC_LUT_Gen_opt(lutSTRUCT *LUT_CONVERTER_sub,treephaseSTRUCT *LUT_3PH_sub,uint32_t  PWM_PERIOD_COUNTER)
{ 
  
  uint32_t i;
  //Il for spazzola un numero di punti pari al fondo scala del timer
  for(i=0;i<PWM_PERIOD_COUNTER;i++)   
  {    
//    if(i>PWM_PERIOD_COUNTER-10)
//    {   
//      i=i+1;
//      i=i-1;      
//    }
    //I punti effettivamente generati sono customizzabili e per questo il passo di scrittura della LUT dipende da un parametro "discr_sin=Sample/PWM_PERIOD_COUNTER"
    // L'indice di scrittura effettivo viene memorizzato in stepLUT 
    
    uint16_t stepLUT;
    
    stepLUT=(uint16_t)((float)i*LUT_3PH_sub->discretization_coefficient);
    
//    LUT_CONVERTER_sub->LUTramp[stepLUT]=i;          //Memorizza una rampa che va da 0 a stepLUT in un array di (n) elementi
    LUT_CONVERTER_sub->LUTramp_2pi[stepLUT]=i*(1.0/(float)PWM_PERIOD_COUNTER)*6.28318531;             //Si normalizza la rampa (0-stepLUT) in (0-2pi)  
    LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]=LUT_3PH_sub->ma*sin((float) LUT_CONVERTER_sub->LUTramp_2pi[stepLUT]);              // Memorizza il seno (-1,1)*ma
//    LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]=LUT_3PH_sub->ma*sin((float)(LUT_CONVERTER_sub->LUTramp[stepLUT]/(float)PWM_PERIOD_COUNTER)*(float)6.283185307179586476925286766559);              // Memorizza il seno (-1,1)*ma
    LUT_CONVERTER_sub->LUTsineNEGfloat[stepLUT]=LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT];
    LUT_CONVERTER_sub->LUTsinefloat[stepLUT]=LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT];
    
#if defined(STDES_PFCBIDIR) || defined(STDES_VIENNARECT) || defined(STDES_PFCBIDIR_REV2)
    if(i<((float)PWM_PERIOD_COUNTER*0.5))
    {  
      LUT_CONVERTER_sub->LUTsinePOS[stepLUT]= (uint16_t)(LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]*PWM_PERIOD_COUNTER);
      LUT_CONVERTER_sub->LUTsineNEG[stepLUT]=(uint16_t)(PWM_PERIOD_COUNTER+1); 
    }
    else
    {  
      LUT_CONVERTER_sub->LUTsinePOS[stepLUT]=0;
      LUT_CONVERTER_sub->LUTsineNEG[stepLUT]= (uint16_t)((LUT_CONVERTER_sub->LUTsineNEGfloat[stepLUT]+1.0)*PWM_PERIOD_COUNTER);
    }
#elif STDES_PFCBIDIR_2L 
    LUT_CONVERTER_sub->LUTsine[stepLUT]=(uint16_t)((LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]+1.0)*0.5*PWM_PERIOD_COUNTER);
    LUT_CONVERTER_sub->LUTsine01[stepLUT]=(float)((LUT_CONVERTER_sub->LUTsinePOSfloat[stepLUT]+1.0)*0.5);
#else    
    SELECT DEFINE
#endif
  } //for 
  LUT_3PH_sub->ramp_offset=(uint16_t)((float)PWM_PERIOD_COUNTER*0.33333333f*LUT_3PH_sub->discretization_coefficient);            // Calcola l'offset di lettura che rappresenta lo sfasamento di 120 gradi da usare nella routine di lettura
}






/**
  * @brief  DPC_LUT_Init:
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void DPC_LUT_Init(treephaseSTRUCT* LUT_STRUCT,uint32_t  PWM_PERIOD_COUNTER,uint32_t RateRead,uint32_t SAMPLE,uint16_t uh_fgrid_Hz_local,float ma_local)
{ 
  LUT_STRUCT->uh_fgrid_Hz=uh_fgrid_Hz_local;                                                                                                                           ///Frequecy Grid emulation (UPS MODE) expressed in Hz
  LUT_STRUCT->ma=ma_local;                                                                                                                                     ///Grid modulation index   (UPS MODE) 
  LUT_STRUCT->discretization_coefficient=(float)SAMPLE/(float)PWM_PERIOD_COUNTER;                                                                             ///Ottiene il coefficiente di disretizzazione funzione del numero di campioni della LUT e il valore massimo della LUT
  LUT_STRUCT->step=(float)((float)(PWM_PERIOD_COUNTER*LUT_STRUCT->discretization_coefficient)/((float)RateRead/(float)LUT_STRUCT->uh_fgrid_Hz));              /// Ottiene il passo dei punti della LUT //il den e periodo  Tim2 / F grid 
  
}




/**
  * @brief  To emulate estimation phase angle provideed from PLL in Inverter standalone mode
  *         
  * @param  LUT_3PH Pointer to treephaseSTRUCT(Typedef) 
  * @param  PERIOD_STEP - LUT read data step period
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void Ramp_Generator(treephaseSTRUCT *LUT_3PH, uint32_t  PERIOD_STEP)
{
  if(LUT_3PH->ramp<(PERIOD_STEP*LUT_3PH->discretization_coefficient))
  {
    LUT_3PH->ramp=LUT_3PH->ramp+LUT_3PH->step;
  }

  else
  {
    LUT_3PH->ramp=0;
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4,GPIO_PIN_RESET);
    
  }
   
  if(LUT_3PH->ramp>=(PERIOD_STEP*LUT_3PH->discretization_coefficient))
  {  
    LUT_3PH->ramp=(PERIOD_STEP*LUT_3PH->discretization_coefficient);
  }  
}



/**
  * @brief  Function used to emulate the phase extimation of PLL, Used in timed task to obtain a discretizated phase ramp
  *         
  * @param  
  *
  * @retval Null 
  *
  * @note Function valid for STM32G4xx microconroller family   
  */
void Ramp_ThreePhGen(treephaseSTRUCT *LUT_3PH,uint32_t  PWM_PERIOD_COUNTER)
{
 uint16_t last_sample=(uint16_t)(PWM_PERIOD_COUNTER*LUT_3PH->discretization_coefficient)-1;
   
  if((uint16_t)LUT_3PH->ramp<last_sample)
  {
    LUT_3PH->ramp=LUT_3PH->ramp+LUT_3PH->step;
  }
  else
  {
    LUT_3PH->ramp=0;
  }
  
  if(LUT_3PH->ramp>=last_sample)
  {  
      LUT_3PH->ramp=(PWM_PERIOD_COUNTER*LUT_3PH->discretization_coefficient);
  } 
  
  LUT_3PH->rampA=(uint16_t)LUT_3PH->ramp+0;
  LUT_3PH->rampB=(uint16_t)LUT_3PH->ramp+LUT_3PH->ramp_offset+LUT_3PH->ramp_offset;
  LUT_3PH->rampC=(uint16_t)LUT_3PH->ramp+LUT_3PH->ramp_offset;
  
  if(LUT_3PH->rampB>=last_sample)
  {
    LUT_3PH->rampB=LUT_3PH->rampB-last_sample; 
  }

  if(LUT_3PH->rampC>=last_sample)
  {
    LUT_3PH->rampC=LUT_3PH->rampC-last_sample; 
  }
}
