/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdlib.h"
#include <stdbool.h>
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
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/*Keypad*/
int keypadCounter= 1;
const char keypad[4][4]={{'1','2','3','A'},
									{'4','5','6','B'},
									{'7','8','9','C'},
									{'*','0','#','D'}};
int column;
int cursore = 0;			
int cntr = 0;
bool clearScreen = true;
extern bool keypadFlag;
char StudentNumber[100];

/*RTC*/
	int i = 0;
	char str[20];
	char str2[20];
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
	
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000); // change &uart1 accordingly
	return ch;
}	

void startPage(){
	setCursor(0,0);
	print("FUM Access Check");
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	printf("uartConnected..\r\n");
}
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
	LiquidCrystal(GPIOE, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	/*initializations*/
	
	/*RTC*/
	RTC_TimeTypeDef mytime;
	RTC_DateTypeDef mydate;
	
	mytime.Hours = 0x0 ;
	mytime.Minutes = 0x0;
	mytime.Seconds= 0x0;
	
	mydate.Year = 0x12;
	mydate.Month = RTC_MONTH_JULY;
	mydate.Date = 0x2;
	mydate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
	
	HAL_RTC_SetTime(&hrtc , &mytime,RTC_FORMAT_BCD);
	HAL_RTC_SetDate(&hrtc , &mydate,RTC_FORMAT_BCD);
	
	//========================================//
	
	/*LCD*/
	startPage();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
//	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
//	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,1);
//	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
//	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,1);
	
  while (1){
		
		/*RTC*/
//		HAL_RTC_GetTime(&hrtc , &mytime,RTC_FORMAT_BCD);
//		HAL_RTC_GetDate(&hrtc , &mydate,RTC_FORMAT_BCD);
//		
//		setCursor(0, 1);
//		if(mydate.WeekDay == 1 ) print("Mon");
//		else if (mydate.WeekDay == 2)	print("TUE");
//		else if (mydate.WeekDay == 3)	print("WED");
//		else if (mydate.WeekDay == 4)	print("THU");
//		else if (mydate.WeekDay == 5)	print("FRI");
//		else if (mydate.WeekDay == 6)	print("SAT");
//		else if (mydate.WeekDay == 7)	print("SUN");
//		
//		setCursor(5, 1);
//		sprintf(str2, "%02d/%02d", mydate.Month , mydate.Date);
//		print(str2);
//		
//		setCursor(11, 1);
//		sprintf(str, "%02d:%02d", mytime.Hours , mytime.Minutes);
//		print(str);
//		
//		HAL_Delay(100);

//===================================================================

		/*Keypad*/
		for(int row=0;row<4;row++){
			column= -1;
			switch(row){
				case 0:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
					break;
				case 1:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,1);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
					break;
				case 2:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
					break;
				case 3:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,1);
					break;
			}
			//debaunce
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)){
				column=0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
				while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6));
			}
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)){
				column=1;
				while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7));
			}
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)){
				column=2;
				while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8));
			}
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)){
				column=3;
				while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9));
			}
			
			if(keypadFlag){
			if(column!=-1){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
				printf("%c",keypad[row][column]);
				
				if(clearScreen == true){
					setCursor(0,0);
					print("                ");
					setCursor(0,1);
					print("                ");
					clearScreen = false;
				}
				
				setCursor(cursore,0);
				cntr=0;
				if(keypad[row][column]=='1'||keypad[row][column]=='2'||keypad[row][column]=='3'||keypad[row][column]=='4'||keypad[row][column]=='5'||
					keypad[row][column]=='6'||keypad[row][column]=='7'||keypad[row][column]=='8'||keypad[row][column]=='9'||keypad[row][column]=='0'){
						setCursor(0,0);
						print("Enter UID :");
						setCursor(cursore++,1);
						StudentNumber[cntr++]=keypad[row][column];
						write(keypad[row][column]);
						
				}else if(keypad[row][column]=='*'){
					printf("finished!\r\n");
					cursore = 0;
					clear();
					startPage();
					break;
				}else if(keypad[row][column]=='#'){
					printf("checkUID!\r\n");
					cursore = 0;
					clear();
					setCursor(0,0);
					print("Cheking UID!");
					break;
				}else if(keypad[row][column]=='A'||keypad[row][column]=='B'||keypad[row][column]=='C'||keypad[row][column]=='D'){
					print("Enter Password :");
					//password check ...
					if(keypad[row][column]=='A'){
						cursore = 0;
						clear();
						setCursor(0,0);
						print("Checking SSID...");
					}else if(keypad[row][column]=='B'){
						cursore = 0;
						clear();
						setCursor(0,0);
						print("Enter UID :");
					}else if(keypad[row][column]=='C'){
						cursore = 0;
						clear();
						setCursor(0,0);
						print("Enter UID :");
					}else if(keypad[row][column]=='D'){
						cursore = 0;
						clear();
						setCursor(0,0);
						print("Restart ...");
					}
				}
			}
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.SynchPrediv = 999;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x10;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL3_Pin COL4_Pin COL1_Pin COL2_Pin */
  GPIO_InitStruct.Pin = COL3_Pin|COL4_Pin|COL1_Pin|COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW3_Pin ROW4_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
