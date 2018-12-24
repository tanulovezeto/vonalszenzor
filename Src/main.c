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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct _TCRT {

	uint32_t data;
	uint32_t channel;
	uint8_t place;
	ADC_HandleTypeDef adc;

} TCRT;

#define LED_latch GPIO_PIN_15
#define TCRT_latch GPIO_PIN_12
#define MUX_A GPIO_PIN_8
#define MUX_B GPIO_PIN_10
#define LED_en1 GPIO_PIN_7
#define LED_en2 GPIO_PIN_8
#define TCRT_en1 GPIO_PIN_6
#define TCRT_en2 GPIO_PIN_9

#define USE_FULL_ASSERT

uint16_t atlag[2], adat[2], spiData[2], trig, kuka, send, target, sum, db;
TCRT line[32];
uint32_t row[32], led[33];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void initLine();
int ADC_value(ADC_HandleTypeDef hadc, uint32_t channel);
void flash();
void mux_switch_y(uint8_t i);
void mux_switch_x(uint8_t i);
void SendSPI();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	initLine();

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

//HAL_GPIO_WritePin(GPIOA, LED_latch, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LED_en1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LED_en2, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, TCRT_en1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, TCRT_en2, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_RESET);

	trig = 400;
	SendSPI();

	adat[1] = 0b0000000000000000;
	adat[0] = 0b0000000000000000;
	HAL_SPI_Transmit(&hspi3, &adat, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, LED_latch, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, LED_latch, GPIO_PIN_RESET);

	while (1) {

		 /*spiData[1] = 0b0000000000000000;
		 spiData[0] = 0b0001000100010001;

		 HAL_SPI_Transmit(&hspi3, &spiData, 2, HAL_MAX_DELAY);
		 HAL_SPI_Transmit(&hspi2, &spiData, 2, HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_RESET);
		 mux_switch_y(0);
		 led[20]=ADC_value(hadc1,ADC_CHANNEL_10);
		 mux_switch_x(0);
		 led[32]=ADC_value(hadc1,ADC_CHANNEL_11);
		 mux_switch_y(0);
		 led[28]=ADC_value(hadc1,ADC_CHANNEL_13);
		 mux_switch_x(0);
		 led[24]=ADC_value(hadc2,ADC_CHANNEL_9);
		 mux_switch_x(0);
		 led[16]=ADC_value(hadc2,ADC_CHANNEL_12);
		 mux_switch_y(0);
		 led[12]=ADC_value(hadc2,ADC_CHANNEL_8);
		 mux_switch_x(0);
		 led[8]=ADC_value(hadc3,ADC_CHANNEL_0);
		 mux_switch_y(0);
		 led[4]=ADC_value(hadc3,ADC_CHANNEL_1);

		 //idáig jó

		 spiData[1] = 0b0000000000000000;
		 spiData[0] = 0b0010001000100010;
		 HAL_SPI_Transmit(&hspi3, &spiData, 2, HAL_MAX_DELAY);
		 HAL_SPI_Transmit(&hspi2, &spiData, 2, HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_RESET);
		 mux_switch_y(1);
		 led[19]=ADC_value(hadc1,ADC_CHANNEL_10);
		 mux_switch_x(1);
		 led[31]=ADC_value(hadc1,ADC_CHANNEL_11);
		 mux_switch_y(1);
		 led[27]=ADC_value(hadc1,ADC_CHANNEL_13);
		 mux_switch_x(1);
		 led[23]=ADC_value(hadc2,ADC_CHANNEL_9);
		 mux_switch_x(1);
		 led[15]=ADC_value(hadc2,ADC_CHANNEL_12);
		 mux_switch_y(1);
		 led[11]=ADC_value(hadc2,ADC_CHANNEL_8);
		 mux_switch_x(1);
		 led[7]=ADC_value(hadc3,ADC_CHANNEL_0);
		 mux_switch_y(1);
		 led[3]=ADC_value(hadc3,ADC_CHANNEL_1);

		 //jó

		 spiData[1] = 0b0000000000000000;
		 spiData[0] = 0b0100010001000100;
		 HAL_SPI_Transmit(&hspi3, &spiData, 2, HAL_MAX_DELAY);
		 HAL_SPI_Transmit(&hspi2, &spiData, 2, HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_RESET);

		 mux_switch_y(2);
		 led[18]=ADC_value(hadc1,ADC_CHANNEL_10);
		 mux_switch_x(2);
		 led[30]=ADC_value(hadc1,ADC_CHANNEL_11);
		 mux_switch_y(2);
		 led[26]=ADC_value(hadc1,ADC_CHANNEL_13);
		 mux_switch_x(2);
		 led[22]=ADC_value(hadc2,ADC_CHANNEL_9);
		 mux_switch_x(2);
		 led[14]=ADC_value(hadc2,ADC_CHANNEL_12);
		 mux_switch_y(2);
		 led[10]=ADC_value(hadc2,ADC_CHANNEL_8);
		 mux_switch_x(2);
		 led[6]=ADC_value(hadc3,ADC_CHANNEL_0);
		 mux_switch_y(2);
		 led[2]=ADC_value(hadc3,ADC_CHANNEL_1);



		 spiData[1] = 0b0000000000000000;
		 spiData[0] = 0b1000100010001000;
		 HAL_SPI_Transmit(&hspi3, &spiData, 2, HAL_MAX_DELAY);
		 HAL_SPI_Transmit(&hspi2, &spiData, 2, HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_RESET);

		 mux_switch_y(3);
		 led[17]=ADC_value(hadc1,ADC_CHANNEL_10);
		 mux_switch_x(3);
		 led[29]=ADC_value(hadc1,ADC_CHANNEL_11);
		 mux_switch_y(3);
		 led[25]=ADC_value(hadc1,ADC_CHANNEL_13);
		 mux_switch_x(3);
		 led[21]=ADC_value(hadc2,ADC_CHANNEL_9);
		 mux_switch_x(3);
		 led[13]=ADC_value(hadc2,ADC_CHANNEL_12);
		 mux_switch_y(3);
		 led[9]=ADC_value(hadc2,ADC_CHANNEL_8);
		 mux_switch_x(3);
		 led[5]=ADC_value(hadc3,ADC_CHANNEL_0);
		 mux_switch_y(3);
		 led[1]=ADC_value(hadc3,ADC_CHANNEL_1);

		 if(led[1]>trig)
		 {
		 adat[0]+=0b0000000010000000;
		 }
		 else
		 {
		 adat[0]&=0b1111111101111111;
		 }


		 if(led[17]>trig)
		 {
		 adat[1]+=0b0000000010000000;
		 }
		 else
		 {
		 adat[1]&=0b1111111101111111;
		 }


		 if(led[2]>trig)
		 {
		 adat[0]+=0b0000000001000000;
		 }
		 else
		 {
		 adat[0]&=0b1111111101111111;
		 }


		 if(led[18]>trig)
		 {
		 adat[1]+=0b0000000001000000;
		 }
		 else
		 {
		 adat[1]&=0b1111111110111111;
		 }
		 if(led[3]>trig)
		 {
		 adat[0]+=0b0000000000100000;
		 }
		 else
		 {
		 adat[0]&=0b1111111111011111;
		 }

		 if(led[19]>trig)
		 {
		 adat[1]+=0b0000000000100000;
		 }
		 else
		 {
		 adat[1]&=0b1111111111011111;
		 }


		 if(led[4]>trig)
		 {
		 adat[0]+=0b0000000000010000;
		 }
		 else
		 {
		 adat[0]&=0b1111111111101111;
		 }


		 if(led[20]>trig)
		 {
		 adat[1]+=0b0000000000010000;
		 }
		 else
		 {
		 adat[1]&=0b1111111111101111;
		 }


		 if(led[5]>trig)
		 {
		 adat[0]+=0b0000000000001000;
		 }
		 else
		 {
		 adat[0]&=0b1111111111110111;
		 }
		 if(led[21]>trig)
		 {
		 adat[1]+=0b0000000000001000;
		 }
		 else
		 {
		 adat[1]&=0b1111111111110111;
		 }
		 if(led[6]>trig)
		 {
		 adat[0]+=0b0000000000000100;
		 }
		 else
		 {
		 adat[0]&=0b1111111111111011;
		 }
		 if(led[22]>trig)
		 {
		 adat[1]+=0b0000000000000100;
		 }
		 else
		 {
		 adat[1]&=0b1111111111111011;
		 }
		 if(led[7]>trig)
		 {
		 adat[0]+=0b0000000000000010;
		 }
		 else
		 {
		 adat[0]&=0b1111111111111101;
		 }
		 if(led[23]>trig)
		 {
		 adat[1]+=0b0000000000000010;
		 }
		 else
		 {
		 adat[1]&=0b1111111111111101;
		 }
		 if(led[8]>trig)
		 {
		 adat[0]+=0b0000000000000001;
		 }
		 else
		 {
		 adat[0]&=0b1111111111111110;
		 }
		 if(led[24]>trig)
		 {
		 adat[1]+=0b0000000000000001;
		 }
		 else
		 {
		 adat[1]&=0b1111111111111110;
		 }
		 if(led[9]>trig)
		 {
		 adat[0]+=0b1000000000000000;
		 }
		 else
		 {
		 adat[0]&=0b0111111111111111;
		 }
		 if(led[25]>trig)
		 {
		 adat[1]+=0b1000000000000000;
		 }
		 else
		 {
		 adat[1]&=0b0111111111111111;
		 }
		 if(led[10]>trig)
		 {
		 adat[0]+=0b0100000000000000;
		 }
		 else
		 {
		 adat[0]&=0b1011111111111111;
		 }
		 if(led[26]>trig)
		 {
		 adat[1]+=0b0100000000000000;
		 }
		 else
		 {
		 adat[1]&=0b1011111111111111;
		 }
		 if(led[11]>trig)
		 {
		 adat[0]+=0b0010000000000000;
		 }
		 else
		 {
		 adat[0]&=0b1101111111111111;
		 }
		 if(led[27]>trig)
		 {
		 adat[1]+=0b0010000000000000;
		 }
		 else
		 {
		 adat[1]&=0b1101111111111111;
		 }
		 if(led[12]>trig)
		 {
		 adat[0]+=0b0001000000000000;
		 }
		 else
		 {
		 adat[0]&=0b1110111111111111;
		 }
		 if(led[28]>trig)
		 {
		 adat[1]+=0b0001000000000000;
		 }
		 else
		 {
		 adat[1]&=0b1110111111111111;
		 }
		 if(led[13]>trig)
		 {
		 adat[0]+=0b0000100000000000;
		 }
		 else
		 {
		 adat[0]&=0b1111011111111111;
		 }
		 if(led[29]>trig)
		 {
		 adat[1]+=0b0000100000000000;
		 }
		 else
		 {
		 adat[1]&=0b1111011111111111;
		 }
		 if(led[14]>trig)
		 {
		 adat[0]+=0b0000010000000000;
		 }
		 else
		 {
		 adat[0]&=0b1111101111111111;
		 }
		 if(led[30]>trig)
		 {
		 adat[1]+=0b0000010000000000;
		 }
		 else
		 {
		 adat[1]&=0b1111101111111111;
		 }
		 if(led[15]>trig)
		 {
		 adat[0]+=0b0000001000000000;
		 }
		 else
		 {
		 adat[0]&=0b1111110111111111;
		 }
		 if(led[31]>trig)
		 {
		 adat[1]+=0b0000001000000000;
		 }
		 else
		 {
		 adat[1]&=0b1111110111111111;
		 }
		 if(led[16]>trig)
		 {
		 adat[0]+=0b0000000100000000;
		 }
		 else
		 {
		 adat[0]&=0b1111111011111111;
		 }
		 if(led[32]>trig)
		 {
		 adat[1]+=0b0000000100000000;
		 }
		 else
		 {
		 adat[1]&=0b1111111011111111;
		 }

			HAL_SPI_Transmit(&hspi3, &adat, 2, 10);
			HAL_GPIO_WritePin(GPIOA, LED_latch, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, LED_latch, GPIO_PIN_RESET);

			adat[0] = 0;
			adat[1] = 0;

			for(int i=0;i<33;i++)
			{
				if(led[i] > trig)
				{
					sum +=i*100;
					db++;
				}
			}

			target = sum/db;
			//SendSPI(); //msg[0] = db, msg[2] = target
			sum = 0;
			db = 0;*/
		;
	}

//while(1)
//{
//	mux_switch_y(0);
//	spiData[1] = 0b0001000100010001;
//	spiData[0] = 0b0001000100010001;
//	flash(0);
//	HAL_Delay(200);
//	for(int i=0;i<3;i++)
//	{
//		HAL_Delay(200);
//		*((uint32_t*)spiData) *= 2;
//		mux_switch_y(i+1);
//		flash(i+1);
//	}
//
//}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	SendSPI();
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* ADC2 init function */
static void MX_ADC2_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ENABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 3;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* ADC3 init function */
static void MX_ADC3_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ENABLE;
	hadc3.Init.ContinuousConvMode = ENABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 2;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI1 init function */
static void MX_SPI1_Init(void) {

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI2 init function */
static void MX_SPI2_Init(void) {

	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI3 init function */
static void MX_SPI3_Init(void) {

	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC6 PC7 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 PA10 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA11 PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int ADC_value(ADC_HandleTypeDef hadc, uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig;

	if (HAL_ADC_Stop(&hadc) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	sConfig.Channel = channel;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	HAL_Delay(1);
	if (HAL_ADC_Start(&hadc) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	return HAL_ADC_GetValue(&hadc);
}

void flash(int h) {
	HAL_SPI_Transmit(&hspi3, &spiData, 2, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi2, &spiData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_SET);
	mux_switch_x(h);
	line[7 - h].data = ADC_value(line[7 - h].adc, line[7 - h].channel);
	mux_switch_y(h);
	line[3 - h].data = ADC_value(line[3 - h].adc, line[3 - h].channel);
	line[11 - h].data = ADC_value(line[11 - h].adc, line[11 - h].channel);
	line[19 - h].data = ADC_value(line[7 - h].adc, line[19 - h].channel);
	mux_switch_x(h);
	line[23 - h].data = ADC_value(line[23 - h].adc, line[23 - h].channel);
	line[31 - h].data = ADC_value(line[31 - h].adc, line[31 - h].channel);
	line[15 - h].data = ADC_value(line[15 - h].adc, line[15 - h].channel);
	mux_switch_y(h);
	line[27 - h].data = ADC_value(line[27 - h].adc, line[27 - h].channel);
	HAL_GPIO_WritePin(GPIOB, TCRT_latch, GPIO_PIN_RESET);
}

uint16_t cmd[5] = { 0 };
uint16_t msg[5] = { 0 };

uint16_t Calc_SPIChksum(uint16_t* buf, uint16_t len)
{
	uint16_t i;
	uint16_t chk = 0x2A;
	for (i = 0; i < len; ++i)
		chk += buf[i];

	return chk;
}

void SendSPI() {
	msg[0] = 21;
	msg[1] = db;
	msg[2] = target;
	msg[3] = 3;
	msg[4] = Calc_SPIChksum(msg, 4);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_IT(&hspi1, msg, cmd, sizeof(msg));
}

void mux_switch_y(uint8_t i) {
switch (i) {
	case 0: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_RESET);
		//HAL_Delay(1);
		break;
	}
	case 1: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_SET);
		//HAL_Delay(1);
		break;
	}
	case 2: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_SET);
		//HAL_Delay(1);
		break;
	}
	case 3: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_RESET);
		//HAL_Delay(1);
		break;
	}
	default: {
		break;
	}
	}
}

void mux_switch_x(uint8_t i) {
	switch (i) {
	case 0: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_SET);
		//HAL_Delay(1);
		break;
	}
	case 1: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_RESET);
		//HAL_Delay(1);
		break;
	}
	case 2: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_RESET);
		//HAL_Delay(1);
		break;
	}
	case 3: {
		HAL_GPIO_WritePin(GPIOA, MUX_A, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MUX_B, GPIO_PIN_SET);
		//HAL_Delay(1);
		break;
	}
	default: {
		break;
	}
	}
}

void initLine() {
	for (int i = 28; i < 32; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_11;
		line[i].place = i;
		line[i].adc = hadc1;
	}

	for (int i = 24; i < 28; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_13;
		line[i].place = i;
		line[i].adc = hadc1;
	}

	for (int i = 20; i < 24; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_9;
		line[i].place = i;
		line[i].adc = hadc2;
	}

	for (int i = 16; i < 20; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_10;
		line[i].place = i;
		line[i].adc = hadc1;
	}

	for (int i = 12; i < 16; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_12;
		line[i].place = i;
		line[i].adc = hadc2;
	}

	for (int i = 8; i < 12; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_8;
		line[i].place = i;
		line[i].adc = hadc2;
	}

	for (int i = 4; i < 8; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_0;
		line[i].place = i;
		line[i].adc = hadc3;
	}

	for (int i = 0; i < 4; i++) {
		line[i].data = 0;
		line[i].channel = ADC_CHANNEL_1;
		line[i].place = i;
		line[i].adc = hadc3;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line) {
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
