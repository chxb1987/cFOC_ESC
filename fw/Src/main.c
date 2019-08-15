
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "fxptMath.h"
#include "cordicTrig.h"
#include "digitalFilters.h"
#include "pi_reg.h"
#include "rampgen.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"
#include "clarke.h"
#include "smopos.h"
#include "smopos_const.h"
#include "volt_calc.h"
#include "speed_est.h"
//#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_SPEED 115200

#define ADC_DATA_BUFF_SIZE 		3
#define TX_BUFF_SIZE 					10
#define STREAM_DBG_SIZE 			255

#define F_SMP 								(48000000.0f / 4095.0f)
#define T_S 									(1.0f / F_SMP)
#define TETA_BASE 						(M_2PI * 30.0f * T_S)	// for 1 period in buffer cycle(256): 1 / 255 *T_S
#define TETA_INC_CONST				(M_2PI * T_S)
#define SPEED_LOOP_PSC				8


/* 		19.5 max phase current
 * 		max_curr * sqrt(2) * R_sense * opAmp_gain = 19.5 * 1.41 * 0.002 * 30 = 1.6497
 *		3102 = 10a = 14.1 amp
 *		992 = -10a = -14.1 amp
 *		20.8V max DC_BUS voltage (3.27/4095) * 6.3694 * 4095 = 20.8845
 */
	
#define MAX_CURR 19.5			// max readeble from ADC
#define MAX_DC_VOLT 20.9			// max readeble from ADC
#define BASE_CURR 10.0			// base
#define BASE_DC_VOLT 12.0			// MAX_DC_VOLT / sqrt(3)
#define BASE_FREQ 100.0
#define POLES 4
#define RS 0.175
#define LS 0.0000127

volatile uint32_t adcData[ADC_DATA_BUFF_SIZE] = {0};
volatile uint8_t txData[TX_BUFF_SIZE] = {0};

/*========== Drive struct field ============*/
 enum driveState 
 {
    PREP,							// 0
    RDY,							// 1
		TUNE_PI,					// 2
    RUN_OPNLOOP,			// 3
    RUN_CLSLOOP,			// 4
		RUN_SPDLOOP,			// 5
		STOP,							// 6
		SHTDWN						// 7
 };

typedef struct{
	volatile _iq iU;
	volatile _iq iV;
	volatile _iq iW;
	volatile _iq offsetZeroCU;
	volatile _iq offsetZeroCV;
}motor_t;

union drive_u
{
	motor_t bldc;
	uint8_t state;
};

union drive_u drive;
/*==================================*/
/*====== DataLog var ===============*/
typedef struct{
	volatile uint8_t txCmplt:1;
	volatile uint8_t startRecord:1;
	volatile uint8_t endRecord:1;
	volatile uint8_t updateCmplt:1;
	
	volatile uint8_t frameCount;
	volatile uint8_t dataPacketCount;
	volatile uint16_t streamCh1[STREAM_DBG_SIZE];
	volatile uint16_t streamCh2[STREAM_DBG_SIZE];
	volatile uint16_t streamCh3[STREAM_DBG_SIZE];
	volatile uint16_t streamCh4[STREAM_DBG_SIZE];
} dataLogVars_t;

dataLogVars_t dataLog;

/*==========================================*/

volatile _iq tetaStep = _IQ(TETA_INC_CONST);
volatile _iq teta;
volatile _iq olSpeedRef = _IQ(TETA_BASE);
volatile _iq refId = _IQ(0.0);
volatile _iq refIq = _IQ(0.0);
volatile _iq refSpeed = _IQ(0.0);
volatile _iq SpeedLoopCount = 0;

/* Digital LPF Init */
FILTER_DATA lpf_iD = FILTER_DEFAULTS;
FILTER_DATA lpf_iQ = FILTER_DEFAULTS;

PI_REG pi_iD = PI_REG_DEFAULTS;
PI_REG pi_iQ = PI_REG_DEFAULTS;
PI_REG pi_spd = PI_REG_DEFAULTS;

//RAMPGEN rg1 = RAMPGEN_DEFAULTS;

IPARK ipark1 = IPARK_DEFAULTS;

CLARKE clark1 = CLARKE_DEFAULTS;

PARK park1 = PARK_DEFAULTS;

PHASEVOLTAGE voltPh = PHASEVOLTAGE_DEFAULTS;

SVGEN svpwm = SVGEN_DEFAULTS;

SMOPOS smo1 = SMOPOS_DEFAULTS;

//SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;

SPEED_ESTIMATION spdest1 = SPEED_ESTIMATION_DEFAULTS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Configure_DMA(void);
void Configure_USART(void);
void StartTransfers(void);
void updateFrame(uint16_t frameNmb);

void escSystemStart(void)
{
	/* TIM3 Config and start. Enable the update/CC3 interrupt
   * Enable channles and set compare
	 * Enable counter 
	 * Force update/CC3 event generation */
  //LL_TIM_EnableIT_CC3(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetAutoReload(TIM3, 2047);
	LL_TIM_OC_SetCompareCH1(TIM3, 0);
	LL_TIM_OC_SetCompareCH2(TIM3, 0);
	LL_TIM_OC_SetCompareCH4(TIM3, 0);
	LL_TIM_OC_SetCompareCH3(TIM3, LL_TIM_GetAutoReload(TIM3));
	LL_TIM_EnableCounter(TIM3);
  LL_TIM_GenerateEvent_CC3(TIM3);
	
  /* Set DMA transfer addresses of source and destination
   * Set DMA transfer size
   * Enable ISR from transfer complete
   * Enable DMA channel */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&adcData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_DATA_BUFF_SIZE);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	
	/* ADC Config and start. Enable the update interrupt. 
	 * Disable DMA transfer before calibration
	 * Start calibration and wait untill that complete
	 * Enable DMA transfer, enable ADC, wait until ADC switch to enable state
	 * if external event used for triger ADC, first start will need software */
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
	LL_ADC_StartCalibration(ADC1);
	while(LL_ADC_IsCalibrationOnGoing(ADC1)){}
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	//LL_ADC_EnableIT_EOS(ADC1);
	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0){}

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_REG_StartConversion(ADC1);
	
}

void newSample_callback(void)
{
	LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
	
	if ( drive.state  == RDY || drive.state  == RUN_OPNLOOP || drive.state  == RUN_CLSLOOP || drive.state  == RUN_SPDLOOP  )
	{
	/*------------- Get ADC result and Clarke transform --------------*/
		
	clark1.As = ( adcData[0] - drive.bldc.offsetZeroCU ) << 13;
	clark1.Bs = ( adcData[1] - drive.bldc.offsetZeroCV ) << 13;
		
	CLARKE_MACRO(clark1);
		
	/*------------- Ramp and sawtooth wave generation -----------------*/
		
	if (drive.state != TUNE_PI )
	{
	if ( tetaStep < olSpeedRef ) tetaStep += _IQ(0.000001);
		if ( tetaStep > olSpeedRef ) tetaStep -= _IQ(0.000001);
	teta += tetaStep;
	if ( teta >= _IQ(M_PI) ) { teta -= _IQ(M_2PI); }
	}
	
	/*------------- Park transform --------------------------*/
	
//	CORDICsincos( _IQtoIQ13(teta), CORDIC_1F, &park1.Sin, &park1.Cos);
//  CORDICatan2sqrt(&angle,&calcSin,qSin, qCos);
	
	if (drive.state == RUN_OPNLOOP )
	{
	park1.Sin = _IQsin(teta);
	park1.Cos = _IQcos(teta);
	}
	if (drive.state == RUN_CLSLOOP)
	{
	park1.Sin = _IQsin(smo1.Theta);
	park1.Cos = _IQcos(smo1.Theta);
	}
	park1.Alpha = clark1.Alpha;
	park1.Beta = clark1.Beta;
	park1.calc(&park1);
	
	/*------------- SpeedLoop --------------------------*/
	
	if (drive.state == RUN_SPDLOOP && SpeedLoopCount == SPEED_LOOP_PSC)
	 {
	  pi_spd.ref = refSpeed;
	  pi_spd.fdb = spdest1.EstimatedSpeed;
	  pi_iD.calc(&pi_spd);
	  SpeedLoopCount=1;
	 }
	else SpeedLoopCount++;

	if(drive.state == PREP || drive.state == RDY || drive.state == STOP )
	{
		pi_spd.intgrl = 0; pi_spd.out = 0;
	}
	/*------------- cumpute current PI -------------------*/
	
//	lpf_iD.x = park1.Ds;
//	Filter_Execute(&lpf_iD);
//	park1.Ds = lpf_iD.y;
//	
//	lpf_iQ.x = park1.Qs;
//	Filter_Execute(&lpf_iQ);
//	park1.Qs = lpf_iQ.y;
	
	pi_iD.ref = refId;
	pi_iD.fdb = park1.Ds;
	pi_iD.calc(&pi_iD);
	
	if ( drive.state == RUN_SPDLOOP ) pi_iQ.ref = pi_spd.out;
	else pi_iQ.ref = refIq;
	pi_iQ.fdb = park1.Qs;
	pi_iQ.calc(&pi_iQ);
	
	if ( drive.state <= RDY )
	{
		pi_iD.intgrl = 0;
		pi_iQ.intgrl = 0;
		pi_spd.intgrl = 0;
	}
	
	/*------------- Inverse Park transform --------------------------*/
	
	ipark1.Ds = pi_iD.out;
	ipark1.Qs = pi_iQ.out;
	ipark1.Sin = park1.Sin;
	ipark1.Cos = park1.Cos;
	ipark1.calc(&ipark1);
	
	/*------------- Phase voltages calc --------------------------*/
	
	voltPh.MfuncV1 = svpwm.Ta;
	voltPh.MfuncV2 = svpwm.Tb;
	voltPh.MfuncV3 = svpwm.Tc;
	voltPh.DcBusVolt = ( adcData[2] << 12 );
	PHASEVOLT_MACRO(voltPh);
	
	/*------------- SMO --------------------------*/
	
	if (drive.state == RUN_CLSLOOP && smo1.Kslide<_IQ(0.15)) smo1.Kslide=smo1.Kslide+_IQ(0.00001);
	// Low Kslide responds better to loop transients
	// Increase Kslide for better torque response after closing the speed loop

	smo1.Ialpha = clark1.Alpha;
	smo1.Ibeta  = clark1.Beta;
	smo1.Valpha = voltPh.Valpha;
	smo1.Vbeta  = voltPh.Vbeta;
	SMO_MACRO(smo1);
	smo1.Theta = _IQmpy2( _IQ14toIQ(smo1.Theta) );
	
	/*------------- Speed estimation --------------------------*/
	
	spdest1.EstimatedTheta = teta;
	SE_MACRO(spdest1);
	
	/*-------------  SVPWM generation --------------------------------*/
	
	svpwm.Ualpha = ipark1.Alpha;
	svpwm.Ubeta = ipark1.Beta;
	
	SVGENDQ_MACRO(svpwm);
	
	LL_TIM_OC_SetCompareCH1(TIM3, (svpwm.Ta >> 14) + 1023);
	LL_TIM_OC_SetCompareCH2(TIM3, (svpwm.Tb >> 14) + 1023);
	LL_TIM_OC_SetCompareCH4(TIM3, (svpwm.Tc >> 14) + 1023);
	
	}
	
	/*----------------------------------------------*/
	
	if (drive.state == PREP)
	{
		/*------------ Estimate zero offset ------------*/
		drive.bldc.iU = adcData[0];
		drive.bldc.iV = adcData[1];
		/* Disable output channels */
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		static uint8_t zeroCnt = 0;
		drive.bldc.offsetZeroCU += drive.bldc.iU;
		drive.bldc.offsetZeroCV += drive.bldc.iV;
		zeroCnt++;
		if (zeroCnt >= 128)	// if complete all samples
		{
			drive.bldc.offsetZeroCU = drive.bldc.offsetZeroCU >> 7; // zeroU >> 7;
			drive.bldc.offsetZeroCV = drive.bldc.offsetZeroCV >> 7; // zeroV >> 7;
			zeroCnt = 0;
			drive.state = RDY;
			/* Enable output channels */
			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		}
	}
	
	/********* USART data loger *************/
	if (dataLog.startRecord){
		dataLog.endRecord = 0;
		dataLog.updateCmplt = 0;
		
		dataLog.streamCh1[dataLog.frameCount] = clark1.Alpha >> 16;
		dataLog.streamCh2[dataLog.frameCount] = clark1.Beta >> 16;
		dataLog.streamCh3[dataLog.frameCount] = voltPh.DcBusVolt >> 16;
		dataLog.streamCh4[dataLog.frameCount] = smo1.Theta >> 16;
		
		dataLog.frameCount++;
		if (dataLog.frameCount >= STREAM_DBG_SIZE){
		dataLog.endRecord = 1;
		dataLog.startRecord = 0;
		dataLog.frameCount = 0;
		}
	}
	
	/********************************************/
	
	LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
}

void txCmplt_callback(void)
{
	//dataLog.txCmplt = 1;
}

void rxCmplt_callback(void){
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	/* Digital LPF Init */
	lpf_iD.Ts = _IQ(T_S);
	lpf_iD.Tf = _IQ(0.00002);
	Filter_Init(&lpf_iD);
	
	lpf_iQ.Ts = _IQ(T_S);
	lpf_iQ.Tf = _IQ(0.00002);
	Filter_Init(&lpf_iQ);

/* Init PI controllers: current DQ, speed */
	pi_iD.Kp = _IQ(0.09);
	pi_iD.Ki = _IQ(0.29);									
  pi_iD.max_out = _IQ(0.75);
  pi_iD.min_out = _IQ(-0.75);    
 	pi_iD.Ts =  _IQ(T_S);
	
	pi_iQ.Kp = _IQ(0.09);	// Kp = L / ( U_DC * T_S )
	pi_iQ.Ki = _IQ(0.29);	// Ki = T_S / ( R / U_DC )									
  pi_iQ.max_out = _IQ(0.75);
  pi_iQ.min_out = _IQ(-0.75);    
 	pi_iQ.Ts =  _IQ(T_S);
	
	pi_spd.Kp = _IQ(0.5);
	pi_spd.Ki = _IQ(T_S * SPEED_LOOP_PSC / 0.2);									
  pi_spd.max_out = _IQ(0.95);
  pi_spd.min_out = _IQ(-0.95);    
 	pi_spd.Ts =  _IQ(T_S);

	// Initialize the SPEED_EST module SMOPOS based speed calculation
  spdest1.K1 = _IQ21( 1 / ( BASE_FREQ * T_S ) );
  spdest1.K2 = _IQ( 1 / ( 1 + T_S * M_2PI * 5 ) );  // Low-pass cut-off frequency
  spdest1.K3 = _IQ(1) - spdest1.K2;
  spdest1.BaseRpm = 120 * ( BASE_FREQ / POLES );
	
	// Initialize the SMOPOS constant module
//	smo1_const.Rs = RS;
//	smo1_const.Ls = LS;
//	smo1_const.Ib = BASE_CURR;
//	smo1_const.Vb = BASE_DC_VOLT;
//	smo1_const.Ts = T_S;
//	SMO_CONST_MACRO(smo1_const);

// 	smo1.Fsmopos = _IQ(smo1_const.Fsmopos);
// 	smo1.Gsmopos = _IQ(smo1_const.Gsmopos);
	smo1.Fsmopos = _IQ(0.0274); 			// Pass parameters to smo1
	smo1.Gsmopos = _IQ(-0.00622); 		// Pass parameters to smo1
	smo1.Kslide = _IQ(0.01);
	smo1.Kslf = _IQ(0.010570739);


	LL_GPIO_SetOutputPin(POWER_STAGE_EN_GPIO_Port, POWER_STAGE_EN_Pin);
	drive.state = PREP;
	
	/*------------- Start periferia ---------------------*/
	
	Configure_USART();
	Configure_DMA();
	escSystemStart();
	StartTransfers();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*--------------- Transmit UART dataLog ----------------------------*/
		
		if (dataLog.endRecord == 1 && LL_USART_IsActiveFlag_TC(USART2) == 1 ){
		updateFrame(dataLog.frameCount);
		dataLog.txCmplt = 0;
    dataLog.frameCount++;
		if (dataLog.frameCount >= STREAM_DBG_SIZE){
			dataLog.updateCmplt = 1;
			dataLog.endRecord = 0;
			dataLog.frameCount = 0;
			dataLog.dataPacketCount++;
			
			if (dataLog.dataPacketCount < 32) dataLog.startRecord = 1;
			else dataLog.dataPacketCount = 0;
			}
		}
		/*-------------------------------------------------------------------*/
		

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}
static void LL_Init(void)
{
  

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  /* SVC_IRQn interrupt configuration */
  NVIC_SetPriority(SVC_IRQn, 0);
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 0);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);

  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PREDIV_DIV_1);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(48000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(48000000);

  LL_RCC_HSI14_EnableADCControl();

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/* USER CODE BEGIN 4 */

void Configure_USART(void)
{

  /* (1) Enable GPIO clock and configures the USART pins **********************/

  /* Enable the peripheral clock of GPIO Port */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_1);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_1);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

  /* (2) Enable USART2 peripheral clock and clock source ****************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /* Set clock source */
  //LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
	//RCC->CFGR3 |=  

  /* (3) Configure USART2 functional parameters ********************************/
  
  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART2);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 48000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 48000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, UART_SPEED); 

  /* (4) Enable USART2 **********************************************************/
  LL_USART_Enable(USART2);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  { 
  }
}


void Configure_DMA(void)
{
  /* DMA1 used for USART2 Transmission and Reception
   */
  /* (1) Enable the clock of DMA1 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* (2) Configure NVIC for DMA transfer complete/error interrupts */
//  NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0);
//  NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

  /* (3) Configure the DMA functional parameters for transmission */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4, 
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | 
                        LL_DMA_PRIORITY_HIGH              | 
                        LL_DMA_MODE_NORMAL                | 
                        LL_DMA_PERIPH_NOINCREMENT         | 
                        LL_DMA_MEMORY_INCREMENT           | 
                        LL_DMA_PDATAALIGN_BYTE            | 
                        LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
                         (uint32_t)txData,
                         LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, TX_BUFF_SIZE);

  /* (4) Configure the DMA functional parameters for reception */
//  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5, 
//                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | 
//                        LL_DMA_PRIORITY_HIGH              | 
//                        LL_DMA_MODE_NORMAL                | 
//                        LL_DMA_PERIPH_NOINCREMENT         | 
//                        LL_DMA_MEMORY_INCREMENT           | 
//                        LL_DMA_PDATAALIGN_BYTE            | 
//                        LL_DMA_MDATAALIGN_BYTE);
//  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
//                         LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
//                         (uint32_t)aRxBuffer,
//                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ubNbDataToReceive);

  /* (5) Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
}

void StartTransfers(void)
{
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
  LL_USART_EnableDMAReq_TX(USART2);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, TX_BUFF_SIZE);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

/* Datalog-virtual oscilloscipe module
* if encode type BigEndian: place HI byte first, LO byte second.
* if LittleEndian - vice versa.
*/
void updateFrame(uint16_t frameNmb){
	
	/* ---------- for serialPlot/Matlab --------- */
	/* Start frame */
	txData[0] = 0x45; 	// 'E' 0100 0101
	txData[1] = 0x5A;		// 'Z'
	/* Channel 1 */
	txData[2] = (uint8_t) _HI(dataLog.streamCh1[frameNmb]);
	txData[3] = (uint8_t) _LO(dataLog.streamCh1[frameNmb]);
	/* Channel 2 */
	txData[4] = (uint8_t) _HI(dataLog.streamCh2[frameNmb]);
	txData[5] = (uint8_t) _LO(dataLog.streamCh2[frameNmb]);
	/* Channel 3 */
	txData[6] = (uint8_t) _HI(dataLog.streamCh3[frameNmb]);
	txData[7] = (uint8_t) _LO(dataLog.streamCh3[frameNmb]);
	/* Channel 4 */
	txData[8] = (uint8_t) _HI(dataLog.streamCh4[frameNmb]);
	txData[9] = (uint8_t) _LO(dataLog.streamCh4[frameNmb]);
	/* ---------------------------------------------- */
	
	StartTransfers();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
