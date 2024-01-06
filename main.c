/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_10

#define D7_PIN GPIOB, GPIO_PIN_4
#define D6_PIN GPIOB, GPIO_PIN_5
#define D5_PIN GPIOB, GPIO_PIN_3
#define D4_PIN GPIOA, GPIO_PIN_10

#define enable_PIN GPIOC, GPIO_PIN_0

#define RS_PIN GPIOC, GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void us_delay(uint16_t time);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t get_DHT11_bit_data(void);
void init_lcd(void);
void send_lcd_data(uint8_t data7, uint8_t data6, uint8_t data5, uint8_t data4);
void display_lcd_data(const char * line);
void return_cursor(void);
void clear_lcd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t presence = 0;
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
  /* USER CODE BEGIN 2 */
  init_lcd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  clear_lcd();
	  display_lcd_data("Temperature: ");


	  HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){  //set a given pin to an output
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}





void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){   //set a given pin to an input
	GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}







void DHT11_Start(void){     //start the DHT11 to start data collection
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);

	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	us_delay(18000);

	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	us_delay(20);

	Set_Pin_Output(DHT11_PORT, DHT11_PIN);
}






uint8_t DHT11_Check_Response(void){  //check that the DHT11 has responded to the STM32 after pulling the line up
	uint8_t response = 0;
	us_delay(40);
	if(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
		us_delay(80);
		if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) response = 1;
		else response = -1;
		}

	while( HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET);

	return response;

}





uint8_t get_DHT11_bit_data(void){ //read a single byte (set of 8 bits) of data

	//each byte goes: Humidity integeral data + Humidity decimal data + Temperature integeral data + Temperature decimal data + checksum
	//(this is the sum of the previous 4 bytes)

	uint8_t j;
	for(uint8_t i = 0; i < 8; i++){

		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET);
		us_delay(40);



		if( HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN == GPIO_PIN_RESET) )
			j &= ~(1<<(7-i));
		else j |= ~(1<<(7-i));



		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET);

	}
	return j;

}


void init_lcd(void){
	send_lcd_data(0, 0, 1, 0);
	clear_lcd();
	return_cursor();
	send_lcd_data(0, 0, 0, 0);
	send_lcd_data(1, 1, 0, 0);
}


void clear_lcd(void){
	send_lcd_data(0,0,0,0);
	send_lcd_data(0,0,0,1);
}

void return_cursor(void){
	send_lcd_data(0,0,0,0);
	send_lcd_data(0,0,1,0);
}


void send_lcd_data(uint8_t data7, uint8_t data6, uint8_t data5, uint8_t data4){

	HAL_GPIO_WritePin(RS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(enable_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(D7_PIN, data7);
	HAL_GPIO_WritePin(D6_PIN, data6);
	HAL_GPIO_WritePin(D5_PIN, data5);
	HAL_GPIO_WritePin(D4_PIN, data4);

	HAL_Delay(100);

	HAL_GPIO_WritePin(enable_PIN, GPIO_PIN_RESET);

	HAL_Delay(100);

}

void display_lcd_data(const char *line){
	HAL_GPIO_WritePin(RS_PIN, GPIO_PIN_SET);
	for(uint8_t i = 0; line[i] != '\0'; i++){

		HAL_GPIO_WritePin(enable_PIN, GPIO_PIN_SET);
		HAL_Delay(1);

		HAL_GPIO_WritePin(D7_PIN, (line[i] & (1 << 7)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D6_PIN, (line[i] & (1 << 6)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D5_PIN, (line[i] & (1 << 5)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D4_PIN, (line[i] & (1 << 4)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(enable_PIN, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(enable_PIN, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(D7_PIN, (line[i] & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D6_PIN, (line[i] & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D5_PIN, (line[i] & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D4_PIN, (line[i] & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(enable_PIN, GPIO_PIN_RESET);
	}


	/*
	 *
	 * HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(enable_PORT, enable_PIN, GPIO_PIN_SET);


	HAL_GPIO_WritePin(D7_PORT, D7_PIN, data7);
	HAL_GPIO_WritePin(D6_PORT, D6_PIN, data6);
	HAL_GPIO_WritePin(D5_PORT, D5_PIN, data5);
	HAL_GPIO_WritePin(D4_PORT, D4_PIN, data4);

	HAL_Delay(100);

	HAL_GPIO_WritePin(enable_PORT, enable_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_RESET);
	*/

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
