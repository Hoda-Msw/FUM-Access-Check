/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdlib.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
const char keypad[4][4]={{'1','2','3','A'},
									{'4','5','6','B'},
									{'7','8','9','C'},
									{'*','0','#','D'}};
int column;
int cursore = 0;			
bool clearScreen = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	
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
			if(column!=-1){
				printf("%c",keypad[row][column]);
				
				if(clearScreen == true){
					setCursor(0,0);
					print("                ");
					setCursor(0,1);
					print("                ");
					clearScreen = false;
				}
				
				setCursor(cursore,0);
				switch(keypad[row][column]){
						case '0' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '1' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '2' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '3' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '4' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '5' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '6' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '7' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '8' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case '9' :
							write(keypad[row][column]);
						  setCursor(cursore++,0);
							break;
						case 'A':
							
							break;
						case 'B':
							
							break;
						case 'C':
							
							break;
						case 'D':
							
							break;
						case '*':
							
							break;
						case '#':
							
							break;
			}	
		}
	}
	
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
