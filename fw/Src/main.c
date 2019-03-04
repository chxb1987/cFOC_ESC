
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
//#include "EventRecorder.h"
#include "cordicTrig.h"
#include "digitalFilters.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADC_DATA_BUFF_SIZE 		3
#define TX_BUFF_SIZE 					10
#define STREAM_DBG_SIZE 			(uint8_t) 255

#define F_SMP 								(48000000.0f / 2048.0f)
#define T_S 									(1.0f / F_SMP)
#define TETA_INCREMENT 				(M_2PI * 92.0f * T_S)	// for 1 period in buffer cycle(256): 1 / 255 *T_S

volatile uint32_t adcData[ADC_DATA_BUFF_SIZE] = {0};
volatile uint8_t txData[TX_BUFF_SIZE] = {0};

volatile _iq angle = _IQ(0.5), calcSin;
volatile _iq teta = 0;


typedef struct{
	volatile uint8_t txCmplt:1;
	volatile uint8_t B:1;
	volatile uint8_t startRecord:1;
	volatile uint8_t endRecord:1;
	volatile uint8_t updateCmplt:1;
	volatile uint8_t F:1;
	volatile uint8_t G:1;
	volatile uint8_t H:1;
} Flags_t;

Flags_t status;

volatile uint16_t streamCh1[STREAM_DBG_SIZE];
volatile uint16_t streamCh2[STREAM_DBG_SIZE];
volatile uint16_t streamCh3[STREAM_DBG_SIZE];
volatile uint16_t streamCh4[STREAM_DBG_SIZE];

volatile uint8_t frameCount = 0;

volatile _iq qSin, qCos;

volatile _iq vteta, vSin, vCos, vangle;

volatile _iq harmInj0 = _IQ(37.0);										// Injected harmonic frequency (relative of base)
volatile _iq harmInj1 = _IQ(56.0);										// Injected harmonic frequency (relative of base)
volatile _iq hmScale0 = _IQ(0.2);											// Injected harmonic magnitude (relative of base)
volatile _iq hmScale1 = _IQ(0.15);										// Injected harmonic magnitude (relative of base)

FILTER_DATA lpf1 = FILTER_DEFAULTS;

_iq ax,by,cz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Configure_DMA(void);
void Configure_USART(void);
void StartTransfers(void);
void updateFrame(uint8_t frameNmb);

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
	LL_TIM_SetAutoReload(TIM3, 1023);
	LL_TIM_OC_SetCompareCH1(TIM3, 0);
	LL_TIM_OC_SetCompareCH2(TIM3, 0);
	LL_TIM_OC_SetCompareCH4(TIM3, 0);
	LL_TIM_OC_SetCompareCH3(TIM3, LL_TIM_GetAutoReload(TIM3) - 1);
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
	//EventStartA(0);
	LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
	static uint16_t ramp = 0;
	ramp+=5;
	if (ramp >= 65500) ramp = 0;
	
		
	teta += _IQ13(TETA_INCREMENT);
	if ( teta >= _IQ13(M_PI) ) { teta = -_IQ13(M_PI); }
	
//	CORDICsincos(teta, CORDIC_1F, &qSin, &qCos);
//  CORDICatan2sqrt(&angle,&calcSin,qSin, qCos);
	
	vteta = _IQ13toIQ(teta);
	//vSin = _IQ13toIQ(qSin);
	//vCos = _IQsin(vteta);
	//vangle = _IQ13toIQ(angle);
	
	static _iq tetaTmp1, tetaTmp2;
	tetaTmp1 += _IQmpy( _IQ(TETA_INCREMENT), harmInj0);
	tetaTmp2 += _IQmpy( _IQ(TETA_INCREMENT), harmInj1);
	if ( tetaTmp1 >= _IQ(M_PI) ) { tetaTmp1 = -_IQ(M_PI); }
	if ( tetaTmp2 >= _IQ(M_PI) ) { tetaTmp2 = -_IQ(M_PI); }
	
	vSin = _IQsin(vteta) + _IQmpy( _IQsin(tetaTmp1), hmScale0 ) + _IQmpy( _IQsin(tetaTmp2), hmScale1 );
	
	lpf1.x = vSin;
	Filter_Execute(&lpf1);
	vangle = lpf1.y;

	
	/********* virtual oscilloscope *************/
	if (status.startRecord){
		status.endRecord = 0;
		status.updateCmplt = 0;
		
		streamCh1[frameCount] = (uint16_t)(vangle >> 19);
		streamCh2[frameCount] = (uint16_t)(vteta >> 19);
		streamCh3[frameCount] = (uint16_t)(vSin >> 19);
		streamCh4[frameCount] = (uint16_t)(tetaTmp1 >> 22);
		
		frameCount++;
		if (frameCount >= STREAM_DBG_SIZE){
		status.endRecord = 1;
		status.startRecord = 0;
		frameCount = 0;
		}
	}
	/********************************************/
	
	LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
	//EventStopA(0);
}

void txCmplt_callback(void)
{
	status.txCmplt = 1;
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
	
	lpf1.Ts = _IQ(T_S);
	lpf1.Tf = _IQ(0.0003183);
	Filter_Init(&lpf1);
	
	/*------------- Start periferia ---------------------*/
	
	Configure_USART();
	Configure_DMA();
	escSystemStart();
	StartTransfers();
	
	//EventRecorderInitialize(EventRecordAll, 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (status.endRecord == 1 && LL_USART_IsActiveFlag_TC(USART2) == 1 ){
		updateFrame(frameCount);
		status.txCmplt = 0;
    frameCount++;
		if (frameCount >= STREAM_DBG_SIZE){
			status.updateCmplt = 1;
			status.endRecord = 0;
			frameCount = 0;
			}
		}
		

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
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200); 

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
                        LL_DMA_PRIORITY_LOW              | 
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

void updateFrame(uint8_t frameNmb){
	
		/* Start frame */
	txData[0] = 65;
	txData[1] = 66;
	/* Channel 1 */
	txData[2] = (uint8_t) _HI(streamCh1[frameNmb]);
	txData[3] = (uint8_t) _LO(streamCh1[frameNmb]);
	/* Channel 2 */
	txData[4] = (uint8_t) _HI(streamCh2[frameNmb]);
	txData[5] = (uint8_t) _LO(streamCh2[frameNmb]);
	/* Channel 3 */
	txData[6] = (uint8_t) _HI(streamCh3[frameNmb]);
	txData[7] = (uint8_t) _LO(streamCh3[frameNmb]);
	/* Channel 4 */
	txData[8] = (uint8_t) _LO(streamCh4[frameNmb]);
	txData[9] = (uint8_t) _LO(streamCh4[frameNmb]);
	
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
