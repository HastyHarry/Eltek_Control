/**
******************************************************************************
* @file           : DPC_Telemetry.c
* @brief          : Telemetry Module
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

#include "stdio.h"

#include "DPC_Telemetry.h"
#include "DPC_Timeout.h"


/**
* @defgroup Private_Variables                                  Private Variables
  * @{
  */

extern uint8_t pDataTx[];
extern uint8_t pDataRx[];
extern uint8_t DataTxLen;
uint8_t CMD_SET_To_Do;
uint8_t ReadyToSend = 0;
uint8_t ReadyToRead = 0;
uint8_t ReadOK = 0;
uint8_t Get_CMD_r = 0;
uint8_t Get_CMD_s = 0;
uint8_t Get_CMD_l = 0;
uint8_t MaxIndex =0;
//Data_Collect Data_C;
void* Data_Collect[15];
uint8_t Data_ColId[15];
//Data_Collect_U8T Data_STR_Collect[2];
uint8_t Component[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Group_Data_Collect GroupDataCollect;
Data_To_Set New_Data;

//void ReadGroupData(void);
/**
*@}
*/


/**
* @defgroup Private_function                                  Private Variables
  * @{
  */
uint8_t CMD_Read(void);
uint8_t CMD_Loop(void);
uint8_t CMD_V_Loop(void);
uint8_t CMD_I_Loop(void);
uint8_t CMD_O_Loop(void);
void TLS_SetParam(void);
uint8_t Ch_To_EX(char Char);
void ClearBufferRx(void);

/**
*@}
*/


/**
* @brief Declare the Rx done flag
*/

/**
* @brief manage the command Load
*/
uint8_t  TLM_CMD_Load(void)
{
uint8_t ValRet =0;
  
  if(pDataRx[0]== 'r'){
    switch(pDataRx[1]){
      case '1':
        Get_CMD_r = 1;
      break;
      case '2':
        Get_CMD_r = 2;
      break;
      case '3':
        Get_CMD_r = 3;
      break;
      case '4':
        Get_CMD_r = 4;
      break;
      case '5':
        Get_CMD_r = 5;
      break;
      case '6':
        Get_CMD_r = 6;
      break;
      case '7':
        Get_CMD_r = 7;
      break;
      case '8':
        Get_CMD_r = 8;
      break;
      case '9':
        Get_CMD_r = 9;
      break;
      case 'a':
        Get_CMD_r = 10;
      break;
      case 'b':
        Get_CMD_r = 11;
      break;
      case 'c':
        Get_CMD_r = 12;
      break;
      case 'd':
        Get_CMD_r = 13;
      break;
      case 'e':
        Get_CMD_r = 14;
      break;
      case 'f':
        Get_CMD_r = 15;
      break;
      case 'g':
        Get_CMD_r = 16;
      break;
      case 'h':
        Get_CMD_r = 17;
      break;
      default :
        ValRet =0;
      break;
    }
  }
  else if(pDataRx[0]== 'w' ){
    if(pDataRx[1]== 'w' ){
    // enable read param + 8 Byte
      ReadyToRead = 1;
      ReadOK = 1;
    }
  }
  else if(pDataRx[0]== 'p' ){
    if(ReadOK){
      TLS_SetParam();
      ReadOK = 0;
      ValRet =1;
    }
  }
  else if(pDataRx[0]== 'l' ){
    switch(pDataRx[1]){
      case 'l':
        Get_CMD_l = 1;
        DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
      break;
      case 'v':
        Get_CMD_l = 2;
        DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
        break;
      case 'i':
        Get_CMD_l = 3;
        DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
      break;
      case 'o':
        Get_CMD_l = 4;
        DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
      break;
      default :
        ValRet =1;
      break;
      }
    }
  //stop Loop command
  else if(pDataRx[0]== 's' ){
    if(pDataRx[1]== 's'){
      Get_CMD_l =0;
      ValRet =1;
    }
    else{
    ValRet =1;
    }
  }
  else{//error invalid command
    ValRet =0;
  }
  return ValRet;
}
/**
*@}
*/

/**
* @brief Declare the Rx done flag
*/
void TLS_SetParam(void)
{
uint8_t temp;
uint8_t Aux;
  temp = Ch_To_EX(pDataRx[1]);
  New_Data.ParamToSet[temp]=0x00;
  New_Data.Num_Param |=(1<<temp);
  for(uint8_t i=2; i<=9;i++){
    Aux = Ch_To_EX(pDataRx[i]);
    New_Data.ParamToSet[temp]|= (Aux<< ((i-2)*4));
  }
}

/**
* @brief Init the Telemetry package
*/
void TLM_Init(void)
{
  ClearBufferRx();  
}
/**
*@}
*/

/**
* @brief manage the read command
*/
uint8_t TLM_Telemetry_FSM(void)
{
static uint8_t retVal=1;

static uint8_t RxDataOld=0;
uint8_t RxDataNew=pDataRx[0];

  if(retVal){
    retVal = 0;
   DPC_TO_Set(TO_TELEM_2, TLM_TIMEOUT);
  }
  if(Get_CMD_r >= 1){
    /// inserire gestione di read command
    if(CMD_Read()==1){
      ReadyToSend = 1;
      Get_CMD_r =0;
      DPC_TO_Set(TO_TELEM_2, TLM_TIMEOUT);
    }
   Get_CMD_r = 0;
  }
  else if(Get_CMD_l == 1){
    if(CMD_Loop()){
      ReadyToSend = 1;
    DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
    DPC_TO_Set(TO_TELEM_2, TLM_TIMEOUT);
    }
  }
  else if(Get_CMD_l == 2){
    if(CMD_V_Loop()){
      ReadyToSend = 1;
    DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
    DPC_TO_Set(TO_TELEM_2, TLM_TIMEOUT);
    }
  }
  else if(Get_CMD_l == 3){
    if(CMD_I_Loop()){
      ReadyToSend = 1;
    DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
    DPC_TO_Set(TO_TELEM_2, TLM_TIMEOUT);
    }
  }
  else if(Get_CMD_l == 4){
    if(CMD_O_Loop()){
      ReadyToSend = 1;
    DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
    DPC_TO_Set(TO_TELEM_2, TLM_TIMEOUT);
    }
  }
  else if(Get_CMD_s){
  /// to be dev
  }
  if(RxDataNew == RxDataOld){
    if(DPC_TO_Check(TO_TELEM_2)==TO_OUT_TOOK){
      retVal = 1;
      ClearBufferRx();
    }
  }
   RxDataOld = RxDataNew;
 return retVal; 
}
/**
*@}
*/

/**
* @brief manage the read command
*/
uint8_t TLM_readyToSend(void)
{ 
  return ReadyToSend;
}
/**
*@}
*/


/**
* @brief manage the read command
*/
uint8_t TLM_readyToRead(void)
{ 
  return ReadyToRead;
}
/**
*@}
*/

/**
* @brief manage the read command
*/
void TLM_DataSended(void)
{ 
  ReadyToSend = 0;
}
/**
*@}
*/

/**
* @brief manage the read command
*/
void TLM_DataReaded(void)
{ 
  ReadyToRead = 0;
}
/**
*@}
*/
/**
*@}
*/
/**
* @brief manage the read command
*/
uint8_t CMD_Read(void)
{
uint8_t RetS = 0; 
uint8_t CMD_n;
  CMD_n = Get_CMD_r-1;
  if(Get_CMD_r >0){
   pDataTx[0] = 'D';
   sprintf((char *)&pDataTx[1],"%0.2x",Data_ColId[CMD_n]);
   pDataTx[3] = ' ';
   pDataTx[4] = ' ';
   pDataTx[5] = ' ';
   pDataTx[6] = ' ';
   pDataTx[7] = ' ';
   pDataTx[8] = ' ';
   pDataTx[9] = ' ';
   pDataTx[10] = ' ';
   if((Data_ColId[CMD_n] == 0x05)||(Data_ColId[CMD_n] == 0x06)){
     sprintf((char *)&pDataTx[3],"%0.8x",*((uint32_t *) Data_Collect[CMD_n]));
   }
   else if((Data_ColId[CMD_n] == 0x32)||(Data_ColId[CMD_n] == 0x31)){
     sprintf((char *)&pDataTx[3],"%0.3d",*((uint8_t *) Data_Collect[CMD_n]));
   }
   else{
    sprintf((char *)&pDataTx[3],"%d",*((uint32_t *) Data_Collect[CMD_n]));
   }
   pDataTx[11] = '\n';
   RetS=1;
  }
  return RetS;
}
/**
*@}
*/

/**
*@}
*/
/**
* @brief manage the all parameters read command loop
*/
uint8_t CMD_Loop(void)
{
uint8_t RetS = 0; 
static uint8_t Index = 0;

 if(DPC_TO_Check(TO_TELEM_1)==TO_OUT_TOOK){
   sprintf((char *)&pDataTx[1],"%0.2x",GroupDataCollect.Data_L_Id[Index]);
//   pDataTx[2] = ' ';
   pDataTx[3] = ' ';
   pDataTx[4] = ' ';
   pDataTx[5] = ' ';
   pDataTx[6] = ' ';
   pDataTx[7] = ' ';
   pDataTx[8] = ' ';
   pDataTx[9] = ' ';
   pDataTx[10] = ' ';
   if(GroupDataCollect.Data_L_Type[Index] == DT_C){
     pDataTx[0] = 'D';
     sprintf((char *)&pDataTx[3],"%0.3d",*((uint8_t *) GroupDataCollect.Data_List[Index]));
   }
   else if(GroupDataCollect.Data_L_Type[Index] == DT_I){
    pDataTx[0] = 'I';
    sprintf((char *)&pDataTx[3],"%d",*((uint32_t *) GroupDataCollect.Data_List[Index]));
   }
   else if(GroupDataCollect.Data_L_Type[Index] == DT_FL){
    pDataTx[0] = 'F';
    sprintf((char *)&pDataTx[3],"%0.8x",*((uint32_t *) GroupDataCollect.Data_List[Index]));
   }
   pDataTx[11] = '\n';
   RetS=1;
   DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
   if(Index++ >= MaxIndex-1){
     Index = 0;
   }
  }
  else{
    RetS = 0;
  }
  return RetS;
}
/**
*@}
*/

/**
*@}
*/
/**
* @brief manage the Voltage parameters read command loop
*/
uint8_t CMD_V_Loop(void)
{
uint8_t RetS = 0; 
static uint8_t Index = 0;
//static uint8_t TempInd = 0;

//  ReadGroupData();
  if((GroupDataCollect.Data_L_Id[Index]>>4 !=0)){
     if(Index++ > MaxIndex){
      Index = 0;
    }
  }
  else{
   if(DPC_TO_Check(TO_TELEM_1)==TO_OUT_TOOK){
     sprintf((char *)&pDataTx[1],"%0.2x",GroupDataCollect.Data_L_Id[Index]);
//     pDataTx[2] = ' ';
     pDataTx[3] = ' ';
     pDataTx[4] = ' ';
     pDataTx[5] = ' ';
     pDataTx[6] = ' ';
     pDataTx[7] = ' ';
     pDataTx[8] = ' ';
     pDataTx[9] = ' ';
     pDataTx[10] = ' ';
     if(GroupDataCollect.Data_L_Type[Index] == DT_I){
      pDataTx[0] = 'I';
      sprintf((char *)&pDataTx[3],"%d",*((uint32_t *) GroupDataCollect.Data_List[Index]));
     }
     else if(GroupDataCollect.Data_L_Type[Index] == DT_FL){
      pDataTx[0] = 'F';
      sprintf((char *)&pDataTx[3],"%0.8x",*((uint32_t *) GroupDataCollect.Data_List[Index]));
     }
     pDataTx[11] = '\n';
     RetS=1;
    DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
     if(Index++ >= MaxIndex-1){
       Index = 0;
     }
    }
    else{
      RetS = 0;
    }
  }
    return RetS;
}
/**
*@}
*/

/**
*@}
*/
/**
* @brief manage the Current parameters read command loop
*/
uint8_t CMD_I_Loop(void)
{
uint8_t RetS = 0; 
static uint8_t Index = 0;
  
  if((GroupDataCollect.Data_L_Id[Index]>>4 !=1)){
     if(Index++ > MaxIndex){
      Index = 0;
    }
  }
  else{
    if(DPC_TO_Check(TO_TELEM_1)==TO_OUT_TOOK){
//     pDataTx[0] = 'D';
   sprintf((char *)&pDataTx[1],"%0.2x",GroupDataCollect.Data_L_Id[Index]);
//     pDataTx[2] = ' ';
     pDataTx[3] = ' ';
     pDataTx[4] = ' ';
     pDataTx[5] = ' ';
     pDataTx[6] = ' ';
     pDataTx[7] = ' ';
     pDataTx[8] = ' ';
     pDataTx[9] = ' ';
     pDataTx[10] = ' ';
     if(GroupDataCollect.Data_L_Type[Index] == DT_I){
        pDataTx[0] = 'I';
        sprintf((char *)&pDataTx[3],"%d",*((uint32_t *) GroupDataCollect.Data_List[Index]));
     }
     pDataTx[11] = '\n';
     RetS=1;
     DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
     if(Index++ >= MaxIndex-1){
       Index = 0;
     }
    }
    else{
      RetS = 0;
    }
  }
    return RetS;
}
/**
*@}
*/

/**
*@}
*/
/**
* @brief manage the Other parameters read command loop
*/
uint8_t CMD_O_Loop(void)
{
uint8_t RetS = 0; 
static uint8_t Index = 0;
  
  if((GroupDataCollect.Data_L_Id[Index]>>4 !=3)){
    if(Index++ > MaxIndex){
      Index = 0;
    }
  }
  else{
    if(DPC_TO_Check(TO_TELEM_1)==TO_OUT_TOOK){
//     pDataTx[0] = 'D';
     sprintf((char *)&pDataTx[1],"%0.2x",GroupDataCollect.Data_L_Id[Index]);
//     pDataTx[2] = ' ';
     pDataTx[3] = ' ';
     pDataTx[4] = ' ';
     pDataTx[5] = ' ';
     pDataTx[6] = ' ';
     pDataTx[7] = ' ';
     pDataTx[8] = ' ';
     pDataTx[9] = ' ';
     pDataTx[10] = ' ';
     if(GroupDataCollect.Data_L_Type[Index] == DT_C){
      pDataTx[0] = 'D';
      sprintf((char *)&pDataTx[3],"%0.3d",*((uint8_t *) GroupDataCollect.Data_List[Index]));
     }
     pDataTx[11] = '\n';
     RetS=1;
    DPC_TO_Set(TO_TELEM_1, TLM_SPEED);
    if(Index++ >= MaxIndex-1){
       Index = 0;
     }
    }
    else{
      RetS = 0;
    }
  }
    return RetS;
}
/**
*@}
*/



Data_To_Set* TLM_NEWData(void)
{
return &New_Data;
}

/**
*@}
*/
/**
* @brief manage the read command
*/
void TLM_Data_Connect(uint32_t* PointToData, DATA_LIST ParamNum, uint8_t DataId, uint8_t DataType)
{
  GroupDataCollect.Data_List[ParamNum] = PointToData;
  GroupDataCollect.Data_L_Id[ParamNum] = DataId;
  GroupDataCollect.Data_L_Type[ParamNum]= DataType;
  MaxIndex++;
  
}
/**
*@}
*/

uint8_t Ch_To_EX(char Char){
uint8_t ValRet =0xff;
  
    switch(Char){
      case '0':
        ValRet = 0x00;
      break;
      case '1':
        ValRet = 0x01;
      break;
      case '2':
        ValRet = 0x02;
      break;
      case '3':
        ValRet = 0x03;
      break;
      case '4':
        ValRet = 0x04;
      break;
      case '5':
        ValRet = 0x05;
      break;
      case '6':
        ValRet = 0x06;
      break;
      case '7':
        ValRet = 0x07;
      break;
      case '8':
        ValRet = 0x08;
      break;
      case '9':
        ValRet = 0x09;
      break;
      case 'A':
        ValRet = 0x0A;
      break;
      case 'B':
        ValRet = 0x0B;
      break;
      case 'C':
        ValRet = 0x0C;
      break;
      case 'D':
        ValRet = 0x0B;
      break;
      case 'E':
        ValRet = 0x0E;
      break;
      case 'F':
        ValRet = 0x0F;
      break;
      case 'g':
        ValRet = 16;
      break;
      case 'h':
        ValRet = 17;
      break;
      default :
        ValRet =0xFF;
      break;
    }
  
  return ValRet;
}


void ClearBufferRx(void)
{
uint8_t index;
  for(index=0;index<=12;index++){
    pDataRx[index] = 0x00;
  }
}

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
