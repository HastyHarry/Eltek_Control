/**
  ******************************************************************************
  * @file           : Transform.c
  * @brief          : Data Collectiona and access interfaces
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
/* Includes ------------------------------------------------------------------*/
#include "DPC_Transforms.h"
#include "DPC_Math.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
extern float aa;
extern float bb;
extern float cc;
extern int aa_int;
extern int bb_int;
extern int cc_int;
int    xx_int;
int    yy_int;
int    zz_int;
int    ss_int;
int    rr_int;
int    tt_int;

/* Private typedef -----------------------------------------------------------*/
const int scale=31;
#define DbToFxP(x) ((x)*(double)(1<<scale))
#define FxPToDb(x) ((double)(x) / (double)(1<<scale))
#define FloToFxP(x) ((x)*(float)(1<<scale))
#define FxPToFlo(x) ((float)(x) / (float)(1<<scale))
#define IntToFxP(x) ((x)<<scale)
#define FxPToInt(x) ((x)>>scale)
//#define MUL(x,y) (((long long)(x)*(long long)(y))>>scale)
//#define MUL(x,y) (((float)(x)*(float)(y))>>scale)
//#define MUL(x,y) (((double)(x)*(long)(y))>>scale)
#define MUL(x,y) ((((x)>>8)*((y)>>8))>>0)

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Park:  PARK function
  * @param  alpha: angle.
  * @param  beta: angle.
  * @param  theta: angle.
  * @param  d:
  * @param  q:
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */  
void Park(float alpha, float beta, float theta,float phi, float *d, float *q){
    float theta_act=theta+phi;
    float cosine = FastCos(theta_act);
    float sine = FastSin(theta_act);
    *d = alpha*cosine + beta*sine;
    *q = -alpha*sine + beta*cosine;
}


/**
  * @brief  InvPark: Park inverse function
  * @param  d,
  * @param  q,
  * @param  phi, phase angle linked to the reference frame [-pi/2 , pi/2]
  * @param  theta: pointer . 
  * @param  alpha: pointer .
  * @param  beta: pointer .  
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void InvPark(float d, float q, float theta, float phi, float *alpha, float *beta){
    float theta_act=theta+phi;
    float cosine = FastCos(theta_act);
    float sine = FastSin(theta_act);
    *alpha = d*cosine - q*sine;
    *beta =  d*sine + q*cosine;
    }


/**
  * @brief  Clarke: Clark function
  * @param  a:
  * @param  b:
  * @param  alpha: pointer.
  * @param  beta: pointer.  
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */  
void Clarke(float a, float b,float c, float *alpha, float *beta){
    *alpha = 0.666f*(a-(0.5f*b)-(0.5f*c));
    *beta = 0.666666*(0.866025*b - 0.866025*c);
    }


/**
  * @brief  InvClarke: Clark inverse function
  * @param  alpha: 
  * @param  beta: 
  * @param  a: pointer
  * @param  b: pointer  
  * @param  c: pointer
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */  
void InvClarke(float alpha, float beta, float *a, float *b, float *c){
    *a = alpha;
    *b = 0.5f*(-alpha + 1.73205080757f*beta);
    *c = 0.5f*(-alpha - 1.73205080757f*beta);
} 


/////////////**
////////////  * @brief  InvClarke_opt: Clark inverse optimized function
////////////  * @param  alpha: 
////////////  * @param  beta: 
////////////  * @param  a: pointer
////////////  * @param  b: pointer  
////////////  * @param  c: pointer
////////////  * 
////////////  * @retval None
////////////  *
////////////  * @note Function valid for STM32G4xx microconroller family  
////////////  */  
////////////void InvClarke_opt(float alpha, float beta, float *a, float *b, float *c){
////////////    float H_alpha=alpha*0.5f;
////////////    float K_beta=0.866025403785;  //1,73.../2
////////////    *a = alpha;
////////////    *b = -H_alpha+K_beta;
////////////    *c = -H_alpha-K_beta;
////////////}


/**
  * @brief  Clarke_Park: Clark Park function
  * @param  a:
  * @param  b:   
  * @param  c:
  * @param  theta:  
  * @param  d: pointer
  * @param  q: pointer  
  * @param  o: pointer
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
void Clarke_Park(float a, float b, float c, float theta,float phi, float *d, float *q,float *o){
  
  
float alpha_sub;
float beta_sub;
float d_sub;
float q_sub;
//float o_sub;  
  
Clarke(a, b, c, &alpha_sub, &beta_sub);
Park(alpha_sub, beta_sub, theta,phi, &d_sub, &q_sub);


*d=d_sub;
*q=q_sub;
//*o=o_sub=0;

    
    
}




///////////////**
//////////////  * @brief  Clarke_Park_Opt: Clark Park opt function
//////////////  * @param  a: 
//////////////  * @param  b:   
//////////////  * @param  c:
//////////////  * @param  theta:  
//////////////  * @param  d: pointer
//////////////  * @param  q: pointer  
//////////////  * @param  o: pointer
//////////////  * 
//////////////  * @retval None
//////////////  *
//////////////  * @note Function valid for STM32G4xx microconroller family  
//////////////  */
//////////////void Clarke_Park_Opt(float a, float b, float c, float theta, float *d, float *q,float *o){
//////////////  
//////////////  float alpha_sub;
//////////////  float beta_sub;
////////////// 
//////////////  float d_sub;
//////////////  float q_sub;
////////////////  float o_sub;
//////////////
//////////////  Clarke(a, b, &alpha_sub, &beta_sub);
//////////////  Park(alpha_sub, beta_sub, theta, &d_sub, &q_sub);
//////////////
//////////////  *d=d_sub;
//////////////  *q=q_sub;
////////////////  *o=o_sub;
//////////////
//////////////}


/**
  * @brief  Run_ClarkePark: Clark Park Run function
  * @param  abc_sub: 
  * @param  theta_sub:  
  * @param  out_cl_pa_sub:
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
void Run_ClarkePark(TRANSFORM_ABC_t* abc_sub,float theta_sub,float phi_sub,TRANSFORM_QDO_t* out_cl_pa_sub ){
 
float pll_d_sub;
float pll_q_sub;
float pll_o_sub;
 
  Clarke_Park(abc_sub->axA, abc_sub->axB, abc_sub->axC,theta_sub,phi_sub,&pll_d_sub,&pll_q_sub,&pll_o_sub);
  
  
  out_cl_pa_sub->axd=pll_d_sub;
  out_cl_pa_sub->axq=pll_q_sub;
  out_cl_pa_sub->axo=pll_o_sub;
}





/////////////**
////////////  * @brief  Run_ClarkePark_Opt: Clark Park opt Run function
////////////  * @param  abc_sub: 
////////////  * @param  theta_sub:  
////////////  * @param  out_cl_pa_sub:
////////////  * 
////////////  * @retval None
////////////  *
////////////  * @note Function valid for STM32G4xx microconroller family  
////////////  */ 
////////////void Run_ClarkePark_Opt(TRANSFORM_CL_PA_ABC* abc_sub,float theta_sub,TRANSFORM_CL_PA_QDO* out_cl_pa_sub ){
//////////// 
////////////float pll_d_sub;
////////////float pll_q_sub;
////////////float pll_o_sub;
//////////// 
////////////  
////////////  Clarke_Park_Opt(abc_sub->phA, abc_sub->phB, abc_sub->phC,theta_sub,&pll_d_sub,&pll_q_sub,&pll_o_sub);
////////////  
////////////  out_cl_pa_sub->axd=pll_d_sub;
////////////  out_cl_pa_sub->axq=pll_q_sub;
////////////  out_cl_pa_sub->axo=pll_o_sub;
////////////}



/**
  * @brief  inv_Clarke_Park: inv Clark Park function
  * @param  d:
  * @param  q:   
  * @param  o:
  * @param  theta:  
  * @param  phi, phase angle linked to the reference frame [-pi/2 , pi/2]
  * @param  a: pointer
  * @param  b: pointer  
  * @param  c,: pointer
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */  
void inv_Clarke_Park(float d, float q, float o, float theta,float phi, float *a, float *b,float *c){
 
float alpha_sub;
float beta_sub;
float a_sub;
float b_sub;
float c_sub;

InvPark(d, q, theta,phi,&alpha_sub, &beta_sub);
InvClarke(alpha_sub, beta_sub, &a_sub, &b_sub, &c_sub);

*a=a_sub;
*b=b_sub;
*c=c_sub;
} 



/////////////////**
////////////////  * @brief  inv_Clarke_Park_opt: inv Clark Park _opt function
////////////////  * @param  d:
////////////////  * @param  q:   
////////////////  * @param  o:
////////////////  * @param  theta:  
////////////////  * @param  phi, phase angle linked to the reference frame [-pi/2 , pi/2]
////////////////  * @param  a: pointer
////////////////  * @param  b: pointer  
////////////////  * @param  c: pointer
////////////////  * 
////////////////  * @retval None
////////////////  *
////////////////  * @note Function valid for STM32G4xx microconroller family  
////////////////  */  
////////////////void inv_Clarke_Park_opt(float d, float q, float o, float theta,float phi, float *a, float *b,float *c){
////////////////
////////////////float alpha_sub;
////////////////float beta_sub;
////////////////float a_sub;
////////////////float b_sub;
////////////////float c_sub;
////////////////
////////////////InvPark(d, q, theta, phi,&alpha_sub, &beta_sub);
////////////////InvClarke_opt(alpha_sub, beta_sub, &a_sub, &b_sub, &c_sub);
////////////////
////////////////*a=a_sub;
////////////////*b=b_sub;
////////////////*c=c_sub;
////////////////} 


/**
  * @brief  Run_Inv_ClarkePark: Inv Clark Park Run function
  * @param  dqo_sub:
  * @param  theta_sub:  
  * @param  out_inv_cl_pa_sub:
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void Run_Inv_ClarkePark(TRANSFORM_QDO_t* dqo_sub,float theta_sub,float phi_sub,TRANSFORM_ABC_t* out_inv_cl_pa_sub ){
 
float a_sub;
float b_sub;
float c_sub;
 
  inv_Clarke_Park(dqo_sub->axd, dqo_sub->axq, dqo_sub->axo,theta_sub,phi_sub,&a_sub,&b_sub,&c_sub);  
  
  out_inv_cl_pa_sub->axA=a_sub;
  out_inv_cl_pa_sub->axB=b_sub;
  out_inv_cl_pa_sub->axC=c_sub;
}


///////////////////**
//////////////////  * @brief  Run_Inv_ClarkePark_opt: Inv Clark Park Run function
//////////////////  * @param  dqo_sub:
//////////////////  * @param  theta_sub:  
//////////////////  * @param  out_inv_cl_pa_sub:
//////////////////  * 
//////////////////  * @retval None
//////////////////  *
//////////////////  * @note Function valid for STM32G4xx microconroller family  
//////////////////  */
//////////////////void Run_Inv_ClarkePark_opt(TRANSFORM_INV_CL_PA_DQO* dqo_sub,float theta_sub,float phi_sub,TRANSFORM_INV_CL_PA_ABC* out_inv_cl_pa_sub ){
////////////////// 
//////////////////float a_sub;
//////////////////float b_sub;
//////////////////float c_sub;
////////////////// 
//////////////////  inv_Clarke_Park_opt(dqo_sub->axd, dqo_sub->axq, dqo_sub->axo,theta_sub,phi_sub,&a_sub,&b_sub,&c_sub);  
//////////////////  
//////////////////  out_inv_cl_pa_sub->phA=a_sub;
//////////////////  out_inv_cl_pa_sub->phB=b_sub;
//////////////////  out_inv_cl_pa_sub->phC=c_sub;
//////////////////}




