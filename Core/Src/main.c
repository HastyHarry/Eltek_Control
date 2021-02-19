/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "hrtim.h"
#include "rng.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DPC_Pid.h"
#include "DPC_Loopctrl.h"
#include "DPC_Datacollector.h"
#include "DPC_Transforms.h"
#include "DPC_PWMConverter.h"
#include "DPC_PLL.h"
#include "DPC_adc_converter.h"
#include "DPC_Telemetry.h"
#include "DPC_Timeout.h"
#include "DPC_Math.h"
#include "DPC_FSM.h"
#include "DPC_Miscellaneous.h"
#include "DPC_Faulterror.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float Service_data[20][500];
uint16_t Service_step;
uint16_t Service_counter;
uint32_t Prev_Saturation;
uint32_t Flag;
uint32_t Flag2;

/*!FSM*/
PC_State_TypeDef PC_State;                                      /*!< */
Run_State_TypeDef FSM_Run_State;                                /*!< */

/*!DeadTime variables*/
uint8_t DTG_bin;                                                /*!< DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR) expressed in HEX*/

/*!Telemetry global variables*/
uint8_t pDataRx[12];                                            /*!< DMA_RX Buffer*/
uint8_t pDataTx[12];                                            /*!< DMA_TX Buffer*/
uint8_t DataTxLen = 12;                                         /*!< TX Lengt Data*/

/*!Reference Frame Transformation variables*/
TRANSFORM_QDO_t V_DQO_CTRL;                                     /*!< */
TRANSFORM_ABC_t V_ABC_CTRL;                                     /*!< */
TRANSFORM_QDO_t Voltage_qdo;                                    /*!< */
TRANSFORM_QDO_t Current_qdo;                                    /*!< */
TRANSFORM_ABC_t Vabc_temp;                                      /*!< */
TRANSFORM_ABC_t Iabc_temp;                                      /*!< */
TRANSFORM_ABC_t Vabc_Phy;                                       /*!< */
TRANSFORM_ABC_t Iabc_Phy;                                       /*!< */
TRANSFORM_QDO_t Voltage_qdo_Phy;                                /*!< */
TRANSFORM_QDO_t Current_qdo_Phy;                                /*!< */

/*!ADC Sensing variables*/
uint32_t p_ADC1_Data[ADC1_CHs];                                 /*!< */
uint32_t p_ADC2_Data[ADC2_CHs];                                 /*!< */

/*!MISC variables*/
DPC_Load_Status_TypeDef Status_Load;                            /*!< */
DPC_Load_TypeDef DPC_Load;                                      /*!< */
DPC_Source_Status_TypeDef Status_Source;                        /*!< */
DPC_Load_Limit_TypeDef DC_Load_Limit;                           /*!< */
DPC_Source_Limit_TypeDef AC_Source_Limit;                       /*!< */
DPC_Source_TypeDef AC_SOURCE;                                   /*!< */

/*!APPL Timers*/
uint32_t APPL_PERIOD_COUNTER;                                   /*!< */
uint32_t APPL2_PERIOD_COUNTER;                                  /*!< */
uint32_t PeriodTIM2;                                            /*!< */
uint32_t PeriodTIM6;                                            /*!< */

/*!PWM Timers*/
uint32_t f_tim_ket_ck;                                          /*!< Represent frequency Internal clock source (tim_ker_ck) expressed in Hz - see: pag-1063 RM0440 Rev1  */
float t_tim_ket_ck;                                             /*!< Used in DeadTime managment [Sec]*/
uint32_t PWM_FREQ_DESIDERED;                                    /*!< Switching Frequency of converter expressed in [Hz]*/
uint32_t PWM_PERIOD_COUNTER  ;                                   /*!< PWM Period Timer Value [Ticks]*/
uint32_t PeriodTIM1;                                            /*!< Period of the TIM1 (PWM Frequency)*/
uint32_t PeriodTIM8;                                            /*!< Period of the TIM8 (PWM Frequency)*/
DPC_PWM_TypeDef tDPC_PWM;                                       /*!< */
static DMA_PWMDUTY_STRUCT DMA_HRTIM_SRC;


/*!PLL*/
float theta_out_pll;                                            /*!< pointer to theta val output variable [Normalized 0-1].*/
PLL_Struct PLL_CONVERTER;                                       /*!< */
PLL_Struct Timestamp_PLL_CONVERTER;                             /*!< */
float omega_out_pll;                                            /*!< */
VoltageAC_qd_PLL_Struct VOLTAGE_AC_qd_IN_NORM;                  /*!< */
STATUS_PLL_TypeDef PLL_Status;                                  /*!< */

/*!ADC cooked*/
DPC_ADC_Conf_TypeDef DPC_ADC_Conf;                              /*!< */
VoltageAC_ADC_NORM_Struct VOLTAGE_ADC_AC_IN_NORM;               /*!< AC Voltage sensing ADC value expressed in [-1,1] */
VoltageAC_ADC_NORM_Struct VOLTAGE_ADC_AC_IN_PHY;                /*!< AC Voltage sensing ADC value expressed in [Volt] */
VoltageAC_ADC_NORM_Struct VOLTAGE_ADC_AC_IN_BITS;               /*!< AC Voltage sensing ADC value expressed in [ADC_12bits] */
CurrentAC_ADC_NORM_Struct CURRENT_ADC_AC_IN_NORM;               /*!< AC Current sensing ADC value expressed in [-1,1] */
CurrentAC_ADC_NORM_Struct CURRENT_ADC_AC_IN_PHY;                /*!< AC Current sensing ADC value expressed in [Ampere] */
CurrentAC_ADC_NORM_Struct CURRENT_ADC_AC_IN_BITS;               /*!< AC Current sensing ADC value expressed in [ADC_12bits] */
VoltageDC_ADC_NORM_Struct VOLTAGE_ADC_DC_IN_NORM;               /*!< DC Voltage sensing ADC value expressed in [-1,1] */
VoltageDC_ADC_NORM_Struct VOLTAGE_ADC_DC_IN_PHY;                /*!< DC Voltage sensing ADC value expressed in [Volt] */
VoltageDC_ADC_NORM_Struct VOLTAGE_ADC_DC_IN_BITS;               /*!< DC Voltage sensing ADC value expressed in [ADC_12bits] */
CurrentDC_ADC_NORM_Struct_t CURRENT_ADC_DC_IN_NORM;             /*!< DC Current sensing ADC value expressed in [-1,1] */
CurrentDC_ADC_NORM_Struct_t CURRENT_ADC_DC_IN_PHY;              /*!< DC Current sensing ADC value expressed in [Ampere] */
CurrentDC_ADC_NORM_Struct_t CURRENT_ADC_DC_IN_BITS;             /*!< DC Current sensing ADC value expressed in [ADC_12bits] */

/*!Control Loop variables*/
PFC_CTRL_t  pPFC_CTRL;                                          /*!< */
PI_STRUCT_t pPI_VDC_CTRL;                                       /*!< */
VOLTAGECTRL_Struct VOLTAGECTRL;                                 /*!< */
CDC_Struct CDC;                                                 /*!< */
INRUSH_STRUCT INRUSH_CTRL;                                      /*!< */
BURST_STRUCT BURST_CTRL;                                        /*!< */
BURST_STRUCT STARTBURST_CTRL;
INRUSH_StatusTypeDef INRUSH_State;                              /*!< */
BURST_StatusTypeDef BURST_State;                                /*!< */
BURST_StatusTypeDef STARTBURST_State;                           /*!< */

/*!DAC Channel Struct Declaration*/
DAC_Channel_STRUCT DAC_CH;                                      /*!< */

/*!LUTs Struct Declaration*/
lutSTRUCT LUT_CONVERTER;                                        /*!< */
treephaseSTRUCT LUT_3PH;                                        /*!< */

/*!Timestamps Declaration*/
FlagStatus Trigger_Timestamp=SET;                               /*!< */


volatile DPC_FAULTERROR_LIST_TypeDef Error;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void DPC_MISC_Analog_Start(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CORDIC_Init();
  MX_ADC1_Init();
  MX_ADC5_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  MX_HRTIM1_Init();
  MX_TIM15_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */
  PWM_PERIOD_COUNTER = 10000;

  DPC_ADC_Init(&DPC_ADC_Conf,G_VAC,B_VAC,G_IAC,B_IAC,G_VDC,B_VDC,G_IDC,B_IDC);

  DPC_MISC_APPL_Timer_Init(APPL_Tim1, RefreshTime_DESIDERED);   //timer setting                                                                                                                                        /// Function used to Init the timers APP_TIM1 (htim2) used in the power application
  DPC_MISC_APPL_Timer_Init(APPL_Tim2, RefreshTime_TO_DESIDERED);                                                                                                                                        /// Function used to Init the timers APP_TIM1 (htim3) used in the power application
  DPC_MISC_APPL_Timer_Init(APPL_Tim3, RefreshTime2_DESIDERED);

  DPC_MISC_Analog_Start();

  DPC_PLL_Init(&PLL_CONVERTER,PLL_KP, PLL_KI, DPC_PLL_TS,PLL_PHI_2pi,PLL_DELTA_F,PLL_FF_Hz,DPC_PLL_SAT_EN,DPC_PLL_PIsat_up,DPC_PLL_PIsat_down);                                                         /// INIT PLL
  DPC_PI_Init(&CDC.pPI_ID_CURR_CTRL,DPC_ID_KP,DPC_ID_KI,DPC_PI_ID_TS,DPC_PI_ID_sat_up,DPC_PI_ID_sat_down,DPC_PI_ID_SAT_EN,DPC_PI_ID_AW_EN,DPC_PI_ID_AWTG);                                              /// INIT PI CURRENT CTRL D
  DPC_PI_Init(&CDC.pPI_IQ_CURR_CTRL,DPC_IQ_KP,DPC_IQ_KI,DPC_PI_IQ_TS,DPC_PI_IQ_sat_up,DPC_PI_IQ_sat_down,DPC_PI_IQ_SAT_EN,DPC_PI_IQ_AW_EN,DPC_PI_IQ_AWTG);                                              /// INIT PI CURRENT CTRL Q
  DPC_PI_Init(&pPI_VDC_CTRL,DPC_VCTRL_KP,DPC_VCTRL_KI,DPC_PI_VDC_TS,DPC_VCTRL_PI_sat_up,DPC_VCTRL_PI_sat_down,DPC_VCTRL_PI_SAT_EN,DPC_VCTRL_PI_AW_EN,DPC_VCTRL_PI_AWTG);                                /// INIT PI VOLTAGE CTRL
  DPC_LPCNTRL_CDC_Init(&CDC,DPC_PLL_OMEGAGRID,DPC_INDUCTOR,CDC_FF_Init,CDC_DEC_INIT,CDC_VDC_FF_INIT);

  DPC_MISC_ACSource_Init(&AC_Source_Limit,DPC_VAC_PK_OV,DPC_VAC_PK_UV,DPC_VAC_PK_UVLO,DPC_VAC_MIN,DPC_IAC_MAX,&DPC_ADC_Conf);                                                                           /// INIT AC_Source
  DPC_MISC_DCLoad_Init(&DC_Load_Limit,DPC_VDC_OV,DPC_VCAP_LIM,DPC_NO_LOAD_CURR,DPC_LOW_LOAD_CURR,DPC_OVER_LOAD_CURR,&DPC_ADC_Conf);

  DPC_LPCNTRL_PFC_Init(&pPFC_CTRL,DPC_CTRL_INIT,DPC_PFC_VDC,&DPC_ADC_Conf);
  DPC_PWM_Init(DPC_BURST_PWM_FREQ,PWM_FREQ,DPC_PWM_INIT,&tDPC_PWM, &DMA_HRTIM_SRC);
//  DPC_FSM_State_Set(DPC_FSM_WAIT);
//
  DPC_MISC_Appl_Timer_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOA, LED_HL1_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(PFC_SW_SRC_GPIO_Port, PFC_SW_SRC_Pin, GPIO_PIN_SET);

	  //DPC_PWM_OutEnable(&tDPC_PWM);
	  //HAL_HRTIM_SimplePWMStart(&hhrtim1, &PWM_Tim1, HRTIM_OUTPUT_TA1);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC345;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void  DPC_MISC_Analog_Start(void){
  HAL_ADC_Start_DMA(&hadc1,p_ADC1_Data,ADC1_CHs);                              ///HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* p_ADC1_Data, uint32_t Length)
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM2)
	{
		 DATA_Acquisition_from_DMA(p_ADC1_Data,p_ADC2_Data); //Pass ADC DMA Data in DATA LAYER

		//start READ variable from DATA LAYER
		ADC_Current_AC_ProcessData((uint32_t*)Read_Curr_GRID(),&CURRENT_ADC_AC_IN_NORM);                    /// Read Current AC from DATA Layer and pass it at CURRENT_ADC_AC_IN_NORM
		ADC2Phy_Current_ProcessData(&DPC_ADC_Conf,(uint32_t*)Read_Curr_GRID(),&CURRENT_ADC_AC_IN_PHY);      /// Read Current AC from DATA Layer and pass it at CURRENT_ADC_AC_IN_PHY
		//end READ variable from DATA LAYER

		Status_Load=DPC_MISC_Check_DCLoad(&DPC_Load,DC_Load_Limit);
		Status_Source=DPC_MISC_CHECK_AC_SOURCE(&AC_SOURCE,AC_Source_Limit,PLL_CONVERTER.pll_theta_out_2pi);  ///Check AC SOURCE state reading AC Voltage and curent

		Iabc_temp=*((TRANSFORM_ABC_t*)&CURRENT_ADC_AC_IN_NORM);                                             /// Sensing Current
		Iabc_Phy=*((TRANSFORM_ABC_t*)&CURRENT_ADC_AC_IN_PHY);                                               /// Sensing Current in Physical dimension

		Run_ClarkePark(&Iabc_temp, DATA_Read_Theta_PLL(),PLL_CONVERTER.pll_phi_2pi,&Current_qdo);
		//Run_ClarkePark(&Iabc_Phy, DATA_Read_Theta_PLL(),PLL_CONVERTER.pll_phi_2pi,&Current_qdo_Phy);

		DATA_CURR_Write_ClarkePark(Current_qdo);  // Current qdo in DATA layer
		//DATA_CURR_Write_ClarkePark(Current_qdo_Phy);  // Current qdo in DATA layer

		PC_State=FSM_Run;
		if (Status_Source!=OVERCURRENT_SOURCE) Status_Source=OK_SOURCE;
		if (Status_Source==OK_SOURCE){
			FSM_Run_State = Run_PFC_Mode;
		}
		else FSM_Run_State = Run_Idle;
//


		if (PC_State==FSM_Run)                                      ///__________FSM_Run________
		{
		  switch(FSM_Run_State){
		  case Run_PFC_Mode:
			DPC_LPCNTRL_PFC_Mode(&pPFC_CTRL,&pPI_VDC_CTRL,&VOLTAGECTRL,&CDC,&V_DQO_CTRL,&Current_qdo,&Voltage_qdo,&VOLTAGE_ADC_DC_IN_PHY); ///PFC CONTROL______
			Run_Inv_ClarkePark(&V_DQO_CTRL,PLL_CONVERTER.pll_theta_out_2pi,PLL_CONVERTER.pll_phi_2pi,&V_ABC_CTRL);                                  /// DQO to ABC Voltag to provide to modulator
			DPC_PWM_OutEnable(&tDPC_PWM);                                                                                                           ///PWM output will be enabled in PFC_Mode
			HAL_GPIO_WritePin(PFC_SW_SRC_GPIO_Port, PFC_SW_SRC_Pin, GPIO_PIN_SET);
			DPC_PWM_Send_Duty_SPWM(&tDPC_PWM,V_ABC_CTRL.axA,V_ABC_CTRL.axB,V_ABC_CTRL.axC,&DMA_HRTIM_SRC);                                                         ///MODULATOR
			  break;
		  case Run_Burst_Mode:
			DPC_LPCNTRL_Burst_Mode((uint32_t*)Read_Volt_DC(),&BURST_CTRL,(uint32_t*)Read_Curr_DC(),&tDPC_PWM);
			DPC_LPCNTRL_PFC_Mode_Reset(&pPI_VDC_CTRL,&CDC);
			break;
		  case Run_Idle:
			//Idle_Mode();
			DPC_PWM_OutDisable();
			DPC_LPCNTRL_PFC_Mode_Reset(&pPI_VDC_CTRL,&CDC);
			break;
		  }
		}
		else if (PC_State==FSM_StartUp_burst)                        ///__________FSM_StartUp_burst__________
		{
		  DPC_LPCNTRL_Burst_Mode((uint32_t*)Read_Volt_DC(),&STARTBURST_CTRL,(uint32_t*)Read_Curr_DC(),&tDPC_PWM);
		  DPC_LPCNTRL_PFC_Mode_Reset(&pPI_VDC_CTRL,&CDC);
		}
		else if(PC_State==FSM_Fault)                                ///FSM_Fault
		{
		  if(Trigger_Timestamp==SET){
		  Timestamp_PLL_CONVERTER=PLL_CONVERTER;
		  Trigger_Timestamp=RESET;
		  }
		}
		if (Status_Source==OK_SOURCE){
			Flag2=1;
		}
		if (Service_step>=500){
			Service_step=0;
//			if (Flag2==1){
//				Service_step=0;
//			}

		}
//		if (DMA_HRTIM_SRC[0]!=0){
//			DMA_HRTIM_DST[2]=1;
//		}


//		Prev_Saturation = DMA_HRTIM_SRC[0];
		Flag = __HAL_HRTIM_GET_FLAG(&hhrtim1,HRTIM_FLAG_FLT1);

//		VOLTAGE_ADC_AC_IN_BITS.phA = (float)(Flag[0]);
		Service_data[0][Service_step] = DMA_HRTIM_SRC.phA;
		Service_data[1][Service_step] = DMA_HRTIM_SRC.phAB;
//		Service_data[2][Service_step] = V_DQO_CTRL.axd;
//		Service_data[3][Service_step] = DMA_HRTIM_SRC[0];

		//Service_data[4][Service_step]=__HAL_HRTIM_GETCOMPARE(&PWM_Tim1,HRTIM_TIMERINDEX_TIMER_A,HRTIM_COMPAREUNIT_1);
		Service_step++;
	}
	else if(htim->Instance == TIM3){
		TimeoutMng();
	}
	else if(htim->Instance == TIM6){




	//start READ variable from DATA LAYER
	ADC_Voltage_AC_ProcessData((uint32_t*)Read_GRID(),&VOLTAGE_ADC_AC_IN_NORM);                         /// Read Voltage AC from DATA Layer and pass it at VOLTAGE_ADC_AC_IN_NORM
	ADC_Voltage_DC_ProcessData((uint32_t*)Read_Volt_DC(),&VOLTAGE_ADC_DC_IN_NORM);                      /// Read Voltage DC from DATA Layer and pass it at VOLTAGE_ADC_DC_IN_NORM
	ADC_Current_DC_ProcessData((uint32_t*)Read_Curr_DC(),&CURRENT_ADC_DC_IN_NORM);                      /// Read Current DC from DATA Layer and pass it at CURRENT_ADC_DC_IN_NORM
	ADC2Phy_DC_Voltage_ProcessData(&DPC_ADC_Conf,(uint32_t*)Read_Volt_DC(),&VOLTAGE_ADC_DC_IN_PHY);     /// Read Voltage AC from DATA Layer and pass it at VOLTAGE_ADC_AC_IN_PHY
	ADC2Phy_Voltage_ProcessData(&DPC_ADC_Conf,(uint32_t*)Read_GRID(),&VOLTAGE_ADC_AC_IN_PHY);           /// Read Voltage DC from DATA Layer and pass it at VOLTAGE_ADC_DC_IN_PHY
	ADC2Phy_DC_Current_ProcessData(&DPC_ADC_Conf,(uint32_t*)Read_Curr_DC(),&CURRENT_ADC_DC_IN_PHY);     /// Read Current DC from DATA Layer and pass it at CURRENT_ADC_DC_IN_PHY
	//end Send QD FRAME data in DATA Layer

//		Service_data[0][Service_step]=VOLTAGE_ADC_AC_IN_NORM.phA;
//		Service_data[1][Service_step]=VOLTAGE_ADC_AC_IN_NORM.phB;
//		Service_data[2][Service_step]=VOLTAGE_ADC_AC_IN_NORM.phC;
//		//Service_data[3][Service_step]=CURRENT_ADC_AC_IN_PHY.phA;
//		//Service_data[4][Service_step]=CURRENT_ADC_AC_IN_PHY.phB;
//		//Service_data[5][Service_step]=CURRENT_ADC_AC_IN_PHY.phC;
//
//		Service_data[6][Service_step]=V_ABC_CTRL.axA;

//		Service_data[8][Service_step]=V_ABC_CTRL.axC;

	//start Auxiliary Data for Frame Transfrormation
	Vabc_temp=*((TRANSFORM_ABC_t*)&VOLTAGE_ADC_AC_IN_NORM);                                             /// Sensing Voltage
	Vabc_Phy=*((TRANSFORM_ABC_t*)&VOLTAGE_ADC_AC_IN_PHY);                                               /// Sensing Voltage in Physical dimension
	//end Auxiliary Data for Frame Transfrormation

	//start Frame Trasformation
	Run_ClarkePark(&Vabc_temp, DATA_Read_Theta_PLL(),PLL_CONVERTER.pll_phi_2pi,&Voltage_qdo);
	Run_ClarkePark(&Vabc_Phy, DATA_Read_Theta_PLL(),PLL_CONVERTER.pll_phi_2pi,&Voltage_qdo_Phy);        ///
	//end Frame Trasformation

	//start Send QD FRAME data in DATA Layer
	DATA_VOLT_Write_ClarkePark(Voltage_qdo);                                                            /// Voltage qdo in DATA layer
	//end Send QD FRAME data in DATA Layer

	VOLTAGE_AC_qd_IN_NORM=*((VoltageAC_qd_PLL_Struct*)&Voltage_qdo);                                    ///Auxiliary Data for PLL
	PLL_Status=DPC_PLL_pllqd_Run(&PLL_CONVERTER,&VOLTAGE_AC_qd_IN_NORM,&theta_out_pll,&omega_out_pll);  ///PLL Phase Extimation
	DATA_Write_Theta_PLL(PLL_CONVERTER.pll_theta_out_2pi);                                              ///Pass Theta to DATA LAYER


	if (VOLTAGE_ADC_DC_IN_PHY.Vdc_tot>=30){
		HAL_GPIO_WritePin(GPIOA, Relay_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, Relay_Pin, GPIO_PIN_RESET);
	}
	}
}

void HAL_HRTIM_Fault1Callback(HRTIM_HandleTypeDef *hhrtim){
	Status_Source=OVERCURRENT_SOURCE;
}
void HAL_HRTIM_Fault3Callback(HRTIM_HandleTypeDef *hhrtim){
	Status_Source=OVERCURRENT_SOURCE;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
