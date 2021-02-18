/**
 * @file    Telemetry.h
 * @author  Smart Power - SRA CL
 * @version V1.0.0
 * @date    18-July-2019
 * @brief   check and set PFC parameters.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stddef.h"

/** @defgroup Exported_Tyoedef        Exported Typedef
 * @{
 */

#define CMD_ERR     0xFF
#define CMD_READY   0x01
#define CMD_NO      0x00   

#define TLM_TIMEOUT 30000
#define TLM_SPEED   200
/* Global Systems Variables define  */

/*voltage*/
#define VinA     0x00
#define VinB     0x01
#define VinC     0x02
#define VdcP     0x03
#define VdcN     0x04
#define V_D      0x05
#define V_Q      0x06   
#define V_O      0x07   
   
   
/*Current*/
#define IinA     0x10
#define IinB     0x11
#define IinC     0x12
#define Idc      0x13

/*Other*/
#define TA       0x30
#define PC_S     0x31
#define F_R_S_S  0x32
#define PFC_CTR_ST 0x33
  
/*Data Type*/
#define DT_C     0x00
#define DT_I     0x01
#define DT_FL    0x02


typedef struct{
  uint32_t* Data_List[20];
  uint8_t Data_L_Id[20];
  uint8_t Data_L_Type[20];
} Group_Data_Collect;


typedef struct{
  uint32_t ParamToSet[16];
  uint8_t Num_Param;
} Data_To_Set;
     
  typedef enum{
    D0 = 0,
    D1,    
    D2,    
    D3,    
    D4,    
    D5,    
    D6,    
    D7,    
    D8,
    D9,
    D10,
    D11,
    D12,
    D13,
    D14,
    
  } DATA_LIST;   
/**
 * @}
 */
/** @defgroup Exported_Functions        Exported Functions
 * @{
 */
uint8_t TLM_Telemetry_FSM(void); 
uint8_t TLM_CMD_Load(void);
uint8_t TLM_readyToSend(void);
uint8_t TLM_readyToRead(void);
void TLM_Data_Connect(uint32_t* PointToData, DATA_LIST ParamNum, uint8_t DataId, uint8_t DataType);
void TLM_Data_Str_Connect(uint8_t* PointToData,uint8_t nEle, DATA_LIST ParamNum);
void TLM_DataSended(void);
void TLM_DataReaded(void);
Data_To_Set* TLM_NEWData(void);
/**
 * @}
 */


#endif  //TELEMETRY

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
