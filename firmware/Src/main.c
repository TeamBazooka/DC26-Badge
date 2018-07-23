
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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

uint8_t rand(void);
void sequence(void);
void sequence_reverse(void);
void sequence_flash_after(void);
void sequence_reverse_flash_after(void);
void flash_all(void);
void flash_hold_sequence(void);
void flash_sequence(void);
void flash_sequence_times(uint8_t times);
void flash_times(uint8_t times);
void flash(GPIO_TypeDef* GPIOx, uint16_t pin);
void off(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define MAX_RAND 500
uint8_t random_values[MAX_RAND] = {
  7, 3, 4, 6, 9, 2, 2, 9, 1, 9, 7, 5, 9, 2, 9, 2, 7, 8, 0, 4,
  5, 5, 0, 3, 6, 8, 0, 10, 0, 9, 0, 8, 6, 6, 8, 2, 10, 4, 6, 10,
  1, 6, 0, 5, 10, 2, 4, 2, 0, 2, 0, 10, 10, 3, 10, 9, 3, 4, 9, 6,
  0, 7, 0, 5, 8, 3, 3, 6, 8, 0, 6, 6, 3, 4, 4, 9, 1, 3, 7, 10,
  9, 4, 8, 9, 4, 4, 8, 10, 3, 9, 3, 9, 2, 5, 2, 3, 8, 4, 8, 10,
  10, 10, 2, 4, 10, 7, 8, 3, 5, 1, 1, 10, 2, 8, 8, 4, 4, 0, 2, 3,
  3, 10, 0, 2, 0, 9, 10, 0, 5, 6, 2, 10, 0, 9, 9, 0, 5, 9, 4, 1,
  4, 4, 8, 9, 7, 8, 0, 2, 10, 2, 1, 1, 4, 9, 9, 6, 6, 7, 1, 8,
  2, 3, 9, 0, 1, 6, 8, 7, 3, 1, 8, 7, 0, 3, 3, 3, 2, 3, 10, 5,
  6, 1, 8, 9, 1, 7, 6, 3, 5, 1, 0, 9, 9, 1, 5, 6, 1, 2, 4, 4,
  9, 0, 6, 2, 0, 7, 4, 7, 7, 2, 3, 7, 0, 9, 7, 5, 0, 6, 6, 5,
  10, 0, 3, 8, 5, 6, 4, 1, 7, 2, 4, 0, 6, 4, 10, 1, 0, 0, 7, 1,
  0, 6, 4, 3, 5, 2, 7, 1, 8, 8, 9, 6, 1, 5, 4, 5, 6, 10, 3, 5,
  3, 10, 5, 10, 6, 10, 0, 7, 5, 10, 7, 5, 3, 3, 6, 0, 10, 0, 4, 7,
  2, 6, 0, 1, 5, 3, 4, 2, 4, 9, 0, 7, 9, 5, 9, 8, 8, 0, 5, 8,
  1, 10, 6, 0, 1, 1, 5, 9, 1, 7, 5, 7, 2, 1, 9, 7, 9, 1, 6, 5,
  9, 3, 4, 0, 3, 8, 4, 0, 10, 6, 3, 3, 6, 4, 8, 2, 9, 8, 3, 3,
  10, 4, 2, 3, 8, 5, 5, 4, 0, 5, 8, 8, 3, 10, 9, 8, 5, 2, 1, 10,
  10, 2, 4, 6, 2, 7, 9, 6, 9, 10, 4, 3, 8, 7, 8, 1, 9, 8, 6, 3,
  7, 1, 5, 9, 0, 6, 8, 1, 8, 1, 3, 3, 0, 0, 7, 1, 1, 7, 5, 6,
  7, 3, 5, 0, 7, 1, 7, 6, 10, 6, 3, 5, 4, 9, 9, 7, 5, 6, 4, 6,
  7, 3, 2, 2, 0, 3, 8, 9, 7, 2, 5, 8, 4, 10, 3, 7, 6, 0, 5, 3,
  1, 7, 10, 10, 1, 10, 3, 3, 5, 8, 0, 6, 5, 9, 5, 1, 2, 1, 9, 1,
  1, 8, 6, 3, 7, 0, 8, 10, 7, 8, 7, 4, 0, 8, 3, 7, 8, 3, 10, 6,
  9, 6, 10, 8, 3, 4, 4, 7, 0, 10, 0, 0, 4, 7, 6, 7, 8, 9, 9, 8
};
uint current_value = 0;

uint8_t rand(void) {
  if(current_value >= MAX_RAND) {
    current_value = 0;
  }
  return random_values[current_value++];
}

void sequence(void) {
  HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_Z_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_K_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_H_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_S_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, SW_SAO_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  off();
}

void sequence_reverse(void) {
  HAL_GPIO_WritePin(GPIOA, LED_S_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_H_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_K_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_Z_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, SW_SAO_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  off();
}

void sequence_flash_after(void) {
  sequence();
  HAL_Delay(250);
  flash(
    GPIOA,
    LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin|LED_S_Pin|SW_SAO_Pin
  );
  HAL_Delay(250);
  flash(
    GPIOA,
    LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin|LED_S_Pin|SW_SAO_Pin
  );
}

void sequence_reverse_flash_after(void) {
  sequence_reverse();
  HAL_Delay(250);
  flash(
    GPIOA,
    LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin|LED_S_Pin|SW_SAO_Pin
  );
  HAL_Delay(250);
  flash(
    GPIOA,
    LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin|LED_S_Pin|SW_SAO_Pin
  );
}

void flash_all(void) {
  flash_times(3);
}

void flash_hold_sequence(void) {
  flash(GPIOA, LED_B_Pin);
  HAL_Delay(250);
  flash(GPIOA, LED_Z_Pin);
  HAL_Delay(250);
  flash(GPIOA, LED_K_Pin);
  HAL_Delay(250);
  flash(GPIOA, LED_H_Pin);
  HAL_Delay(250);
  flash(GPIOA, LED_S_Pin);
  HAL_Delay(250);
  flash(GPIOA, SW_SAO_Pin);
  HAL_Delay(250);
  off();
}

void flash_sequence(void) {
  flash(GPIOA, LED_B_Pin);
  flash(GPIOA, LED_Z_Pin);
  flash(GPIOA, LED_K_Pin);
  flash(GPIOA, LED_H_Pin);
  flash(GPIOA, LED_S_Pin);
  flash(GPIOA, SW_SAO_Pin);
}

void flash_sequence_times(uint8_t times) {
  for(uint8_t ii=times;ii>=0;ii--) {
    flash_sequence();
  }
}

void flash_times(uint8_t times) {
  for(uint8_t ii=times;ii>=0;ii--) {
    flash(
      GPIOA,
      LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin|LED_S_Pin|SW_SAO_Pin
    );
    HAL_Delay(100);
  }
}

void flash(GPIO_TypeDef* GPIOx, uint16_t pin) {
  HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
}

void off(void) {
  HAL_GPIO_WritePin(
    GPIOA,
    LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin|LED_S_Pin|SW_SAO_Pin,
    GPIO_PIN_SET
  );
}

/**
  * @brief  The application loop call.
  *
  * @retval None
  */
void loop(void) {
  int random_value = rand();
  switch(random_value) {
    case 0:
      flash_all();
      break;
    case 1:
      sequence();
      break;
    case 2:
      sequence_reverse();
      break;
    case 3:
      sequence_flash_after();
      break;
    case 4:
      sequence_reverse_flash_after();
      break;
    case 5:
      flash_hold_sequence();
      break;
    case 6:
      flash_sequence();
      break;
    case 7:
    case 8:
    case 9:
    case 10:
      flash_sequence_times(5);
      flash_times(10);
      flash_sequence_times(10);
      break;
  }
  off();
  HAL_Delay(1000 * 60 * rand());
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    loop();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> I2C1_SCL
     PA10   ------> I2C1_SDA
     PA13   ------> LPUART1_RX
     PA14   ------> LPUART1_TX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin
                          |LED_S_Pin|SW_SAO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_B_Pin LED_Z_Pin LED_K_Pin LED_H_Pin
                           LED_S_Pin SW_SAO_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_Z_Pin|LED_K_Pin|LED_H_Pin
                          |LED_S_Pin|SW_SAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
