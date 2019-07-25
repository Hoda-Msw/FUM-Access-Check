/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdlib.h"
#include <stdbool.h>
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*RTC vars*/
int i = 0;
char str[20];
char str2[20];
RTC_TimeTypeDef mytime;
RTC_DateTypeDef mydate;
extern bool RTCFlag;

/*Keypad vars*/
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
bool uidFlag = true;
bool passFlag = false;					
char StudentNumber[100];
char password[100];
									
/*E2PROM vars*/
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {5, 6, 7};
uint16_t VarValue,VarDataTmp = 0;
unsigned char E2PROMstr[1];
char received[100];
uint16_t E2PROMCounter = 0;
uint16_t E2PROMBuffer[100];
bool E2PROMFlag = false;
bool ESPFlag = false;
/*flags*/
extern bool resetFlag;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId RTCHandle;
osThreadId KeypadHandle;
osThreadId LCDHandle;
osThreadId EEPROMHandle;
osThreadId ESPHandle;
osTimerId myTimer01Handle;
osTimerId myTimer02Handle;
osSemaphoreId UIDSemaphoreHandle;
/* USER CODE BEGIN PV */
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
void StartDefaultTask(void const * argument);
void RTCController(void const * argument);
void keypadController(void const * argument);
void LCDController(void const * argument);
void E2PROMController(void const * argument);
void WebPanelController(void const * argument);
void Callback01(void const * argument);
void Callback02(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RTCInitialization(){
	mytime.Hours = 17 ;
	mytime.Minutes = 37;
	mytime.Seconds= 5;
	
	mydate.Year = 19;
	mydate.Month = RTC_MONTH_JULY;
	mydate.Date = 22;
	mydate.WeekDay = RTC_WEEKDAY_MONDAY;
	
	HAL_RTC_SetTime(&hrtc , &mytime,RTC_FORMAT_BCD);
	HAL_RTC_SetDate(&hrtc , &mydate,RTC_FORMAT_BCD);
}

void setRTC(){
		RTCInitialization();
		setCursor(0, 1);
		if(mydate.WeekDay == 1 ) print("Mon");
		else if (mydate.WeekDay == 2)	print("TUE");
		else if (mydate.WeekDay == 3)	print("WED");
		else if (mydate.WeekDay == 4)	print("THU");
		else if (mydate.WeekDay == 5)	print("FRI");
		else if (mydate.WeekDay == 6)	print("SAT");
		else if (mydate.WeekDay == 7)	print("SUN");
		
		setCursor(5, 1);
		sprintf(str2, "%02d/%02d", mydate.Month , mydate.Date);
		print(str2);
		
		setCursor(11, 1);
		sprintf(str, "%02d:%02d", mytime.Hours , mytime.Minutes);
		print(str);
		
    osDelay(500);
}

//void manageLCD(){
//	setCursor(0,0);
//	print("FUM Access Check");
//	
//	osDelay(500);
//}

void startPage(){
	setCursor(0,0);
	print("FUM Access Check");
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//	printf("uartConnected..\r\n");
	if(RTCFlag){
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
		RTCFlag = false;
		setRTC();
		osTimerStart(myTimer01Handle, 100);
	}
}

void eepromGetId(){
	HAL_FLASH_Unlock();
	/* EEPROM Init */
	if( EE_Init() != EE_OK ){
		Error_Handler();
	}
	EE_ReadVariable(VirtAddVarTab[1], &E2PROMCounter);
	if(E2PROMCounter == 0xFFFF){
		E2PROMCounter = 0;
		EE_WriteVariable(VirtAddVarTab[1], E2PROMCounter);
	}
//	  for(int i = 0; i < E2PROMCounter; i++){
//			EE_ReadVariable(VirtAddVarTab[0]+i, (uint16_t *)&E2PROMBuffer[i]);
//	  }
//	  HAL_UART_Transmit(&huart2, (uint8_t *)E2PROMBuffer, sizeof(E2PROMBuffer), 100);
//	  
	  HAL_FLASH_Lock();
	if(E2PROMFlag){
			HAL_FLASH_Unlock();
	
			for(int i = 0; i < 10; i++){
				if(EE_WriteVariable(VirtAddVarTab[0] + E2PROMCounter++, StudentNumber[i]) != HAL_OK){
					Error_Handler();
				}
				EE_WriteVariable(VirtAddVarTab[1], E2PROMCounter);
			}
			
			EE_WriteVariable(VirtAddVarTab[1], E2PROMCounter);
			HAL_FLASH_Lock();
		}
}

void manageKeypad(){
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
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
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
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
//					printf("%c", keypad[row][column]);
				
					if(clearScreen == true){
						setCursor(0,0);
						print("                ");
						setCursor(0,1);
						print("                ");
						clearScreen = false;
					}
//					clear();
					setCursor(cursore,0);
					cntr=0;
					if(keypad[row][column]=='1'||keypad[row][column]=='2'||keypad[row][column]=='3'||keypad[row][column]=='4'||keypad[row][column]=='5'||
						keypad[row][column]=='6'||keypad[row][column]=='7'||keypad[row][column]=='8'||keypad[row][column]=='9'||keypad[row][column]=='0'){
//							if(uidFlag){
							setCursor(0,0);
							print("Enter UID :");
							setCursor(cursore++,1);
							StudentNumber[cntr++]=keypad[row][column];
							write(keypad[row][column]);
//							}
							
						
					}else if(keypad[row][column]=='*'){
						printf("finished!\r\n");
						clearScreen = true;
						cursore = 0;
						clear();
						RTCFlag = true;
						startPage();
						break;
					}else if(keypad[row][column]=='#'){
						printf("checkUID!\r\n");
						clearScreen = true;
						cursore = 0;
						clear();
						setCursor(0,0);
//						uidFlag = false;
						print("Cheking UID!");
						HAL_Delay(100);
						clear();
						setCursor(0,0);
						print("unauthorized");
						HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
						HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
						HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
						HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
						osTimerStart(myTimer02Handle, 10);
						break;
					}else if(keypad[row][column]=='A'||keypad[row][column]=='B'||keypad[row][column]=='C'||keypad[row][column]=='D'){
						clear();
						setCursor(0,0);
						print("Enter Password :");
						HAL_Delay(100);
//						if(passFlag){
//							password[cntr++]=keypad[row][column];
//							write(keypad[row][column]);
//						}
						
						//password check ...
						if(keypad[row][column]=='A'){
							cursore = 0;
							clear();
							setCursor(0,0);
							ESPFlag = true;
							print("Checking SSID...");
						}else if(keypad[row][column]=='B'){
							cursore = 0;
							clear();
							setCursor(0,0);
							print("saving..");
							E2PROMFlag = true;
							eepromGetId();
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
							HAL_Delay(50);
							HAL_NVIC_SystemReset();
						}
					}
				}
			}
		}
}

//void manageEEPROM(){
//	/* Unlock the Flash Program Erase controller */
//	  HAL_FLASH_Unlock();
//	  
//	  /* EEPROM Init */
//	  if( EE_Init() != EE_OK){
//			Error_Handler();
//	  }
//	  
//	  EE_ReadVariable(VirtAddVarTab[1], &E2PROMCounter);
//	  
//	  if(E2PROMCounter == 0xFFFF){
//			E2PROMCounter = 0;
//			EE_WriteVariable(VirtAddVarTab[1], E2PROMCounter);
//	  }
//	  
//	  for(int i = 0; i < E2PROMCounter; i++){
//			EE_ReadVariable(VirtAddVarTab[0]+i, (uint16_t *)&E2PROMBuffer[i]);
//	  }
//	  HAL_UART_Transmit(&huart2, (uint8_t *)E2PROMBuffer, sizeof(E2PROMBuffer), 100);
//	  
//	  HAL_FLASH_Lock();
//	  
//	  HAL_UART_Receive_IT(&huart2, E2PROMstr, 1);
//		
//		if(E2PROMFlag){
//			HAL_FLASH_Unlock();
//	
//			E2PROMBuffer[E2PROMCounter]=E2PROMstr[0];
//	
//			for(int i = 0; i < 10; i++){
//				if(EE_WriteVariable(VirtAddVarTab[0] + E2PROMCounter++, StudentNumber[i]) != HAL_OK){
//					Error_Handler();
//				}
//			}
//			EE_WriteVariable(VirtAddVarTab[1], E2PROMCounter);
//	
//			HAL_FLASH_Lock();
//			HAL_UART_Receive_IT(&huart2,E2PROMstr,1);
//		}
//}



//void resetFactory(){
//	if(resetFlag){
//		HAL_NVIC_SystemReset();
//		keypadCounter= 1;
//		cursore = 0;			
//		cntr = 0;
//		clearScreen = true;
//		VarValue = 0;
//		VarDataTmp = 0;
//		E2PROMCounter = 0;
//		E2PROMFlag = false;		
//		keypadFlag = false;
//		RTCFlag = true;
//		resetFlag = false;
//	}	
//}

void atCommound(){
	printf("AT\r\n");
	osDelay(700);
	printf("AT+CWMODE=1\r\n");
	osDelay(2700);
	printf("AT+AT+CIFSR\r\n");
	osDelay(2700);
	printf("AT+CWJAP=\"hodhod\",\"12345678\"\r\n");
	osDelay(2700);
	printf("AT+CWJAP=?\r\n");
	osDelay(2700);
	printf("AT+CIPMUX=1\r\n");
	osDelay(2700);
	printf("AT+CIPSERVER=1,80\r\n");
	osDelay(2700);
//	printf("AT+CWJAP=\"hodhod\",\"12345678\",4,0\r\n");
//	osDelay(2700);
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
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  /* USER CODE BEGIN 2 */
	
	/*RTC initialization*/
	
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UIDSemaphore */
  osSemaphoreDef(UIDSemaphore);
  UIDSemaphoreHandle = osSemaphoreCreate(osSemaphore(UIDSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* definition and creation of myTimer02 */
  osTimerDef(myTimer02, Callback02);
  myTimer02Handle = osTimerCreate(osTimer(myTimer02), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RTC */
  osThreadDef(RTC, RTCController, osPriorityBelowNormal, 0, 128);
  RTCHandle = osThreadCreate(osThread(RTC), NULL);

  /* definition and creation of Keypad */
  osThreadDef(Keypad, keypadController, osPriorityNormal, 0, 128);
  KeypadHandle = osThreadCreate(osThread(Keypad), NULL);

  /* definition and creation of LCD */
  osThreadDef(LCD, LCDController, osPriorityNormal, 0, 128);
  LCDHandle = osThreadCreate(osThread(LCD), NULL);

  /* definition and creation of EEPROM */
  osThreadDef(EEPROM, E2PROMController, osPriorityHigh, 0, 128);
  EEPROMHandle = osThreadCreate(osThread(EEPROM), NULL);

  /* definition and creation of ESP */
  osThreadDef(ESP, WebPanelController, osPriorityNormal, 0, 128);
  ESPHandle = osThreadCreate(osThread(ESP), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){	
		
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
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL2_Pin COL3_Pin COL0_Pin COL1_Pin */
  GPIO_InitStruct.Pin = COL2_Pin|COL3_Pin|COL0_Pin|COL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;){
		
		
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_RTCController */
/**
* @brief Function implementing the RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTCController */
void RTCController(void const * argument)
{
  /* USER CODE BEGIN RTCController */
//	RTCInitialization();
  /* Infinite loop */
  for(;;){
		if(RTCFlag){
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
			RTCFlag = false;
			setRTC();
			osTimerStart(myTimer01Handle, 100);
		}
  }
  /* USER CODE END RTCController */
}

/* USER CODE BEGIN Header_keypadController */
/**
* @brief Function implementing the Keypad thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_keypadController */
void keypadController(void const * argument)
{
  /* USER CODE BEGIN keypadController */
  /* Infinite loop */
  for(;;){
		manageKeypad();
    osDelay(1);
  }
  /* USER CODE END keypadController */
}

/* USER CODE BEGIN Header_LCDController */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCDController */
void LCDController(void const * argument)
{
  /* USER CODE BEGIN LCDController */
	
  /* Infinite loop */
	startPage();
  for(;;){
		
  }
  /* USER CODE END LCDController */
}

/* USER CODE BEGIN Header_E2PROMController */
/**
* @brief Function implementing the EEPROM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_E2PROMController */
void E2PROMController(void const * argument)
{
  /* USER CODE BEGIN E2PROMController */
//	manageEEPROM();
	eepromGetId();
	E2PROMFlag = false;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END E2PROMController */
}

/* USER CODE BEGIN Header_WebPanelController */
/**
* @brief Function implementing the ESP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WebPanelController */
void WebPanelController(void const * argument)
{
	if(ESPFlag){
		atCommound();
		ESPFlag = false;
	}
  /* USER CODE BEGIN WebPanelController */
  /* Infinite loop */
  for(;;){
		
		
    osDelay(1);
  }
  /* USER CODE END WebPanelController */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
	RTCFlag = true;
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
  osTimerStop(myTimer01Handle);
  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void const * argument)
{
  /* USER CODE BEGIN Callback02 */
	printf("hiii");
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	osTimerStop(myTimer02Handle);
  /* USER CODE END Callback02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
