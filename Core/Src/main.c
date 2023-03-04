/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "Lora.h"
#include "ST7735_SPI.h"
#include "fonts.h"
#include "LOGO.h"
#include "function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APN "m3-world"
#define API_KEY "D1JFHWN9COHH3Z3N"
#define URL "http://api.thingspeak.com/update?key=" API_KEY "&"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
LoRa myLoRa;
uint8_t LoRa_status, a = 0;
uint8_t Rxbuffer[10];
uint16_t Data16_Humi[2], Data16_Temp[2];
uint32_t Data32_Humi, Data32_Temp;
float Humi1, Temp1, Humi2, Temp2;
char lcd_Send[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendATCommand(char* command, char* response);
void configureGPRS(void);
void sendDataToThingSpeak(float tem, float hum, int i);
void Lora_setup(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DIO0_Pin)
	{
		LoRa_receive(&myLoRa, Rxbuffer, 10);
		if(Rxbuffer[0] == 0x01)
		{
			if (Rxbuffer[1] == 0x02)  // Temp
			{
				Data16_Temp[0] = Rxbuffer[2]<<8 | Rxbuffer[3];
				Data16_Temp[1] = Rxbuffer[4]<<8 | Rxbuffer[5];
				Data32_Temp = Data16_Temp[1]<<16 | Data16_Temp[0];
				Temp1 = unpack754_32(Data32_Temp);
				Data16_Humi[0] = Rxbuffer[6]<<8 | Rxbuffer[7];
				Data16_Humi[1] = Rxbuffer[8]<<8 | Rxbuffer[9];
				Data32_Humi = Data16_Humi[1]<<16 | Data16_Humi[0];
				Humi1 = unpack754_32(Data32_Humi);
				sendDataToThingSpeak(Temp1, Humi1, 1);
			}
		}
		if(Rxbuffer[0] == 0x02)
		{
			if (Rxbuffer[1] == 0x02)  // Temp
			{
				Data16_Temp[0] = Rxbuffer[2]<<8 | Rxbuffer[3];
				Data16_Temp[1] = Rxbuffer[4]<<8 | Rxbuffer[5];
				Data32_Temp = Data16_Temp[1]<<16 | Data16_Temp[0];
				Temp2 = unpack754_32(Data32_Temp);
				Data16_Humi[0] = Rxbuffer[6]<<8 | Rxbuffer[7];
				Data16_Humi[1] = Rxbuffer[8]<<8 | Rxbuffer[9];
				Data32_Humi = Data16_Humi[1]<<16 | Data16_Humi[0];
				Humi2 = unpack754_32(Data32_Humi);
				sendDataToThingSpeak(Temp2, Humi2, 3);
			}
		}
	}
	if (GPIO_Pin == GPIO_PIN_4)
	{
		a = 1 - a;
	}


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  configureGPRS();
  Lora_setup();
  LoRa_init(&myLoRa);
  LoRa_startReceiving(&myLoRa);
  ST7735_Init();
  ST7735_FillScreen(ST7735_WHITE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (a == 1)
	  {
		  	ST7735_WriteString(20, 0,"Station 1" , Font_11x18, ST7735_BLACK, ST7735_WHITE);
		  	sprintf(lcd_Send,"Temp:%.2f oC",Temp1);
		  	ST7735_WriteString(0, 30,lcd_Send , Font_11x18, ST7735_BLACK, ST7735_WHITE);
		  	sprintf(lcd_Send,"Humi:%.2f ppm",Humi1);
		  	ST7735_WriteString(0, 60,lcd_Send , Font_11x18, ST7735_BLACK, ST7735_WHITE);
	  }
	  if (a == 0)
	  {
		  	ST7735_WriteString(20, 0,"Station 2" , Font_11x18, ST7735_BLACK, ST7735_WHITE);
		  	sprintf(lcd_Send,"Temp:%.2f oC",Temp2);
		  	ST7735_WriteString(0, 30,lcd_Send , Font_11x18, ST7735_BLACK, ST7735_WHITE);
		  	sprintf(lcd_Send,"Humi:%.2f ppm",Humi2);
		  	ST7735_WriteString(0, 60,lcd_Send , Font_11x18, ST7735_BLACK, ST7735_WHITE);
	  }
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
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
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_Pin RST_Pin PB12 PB5 */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin|GPIO_PIN_12|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Lora_setup(void)
{
	  myLoRa.CS_port         = NSS_GPIO_Port;
	  myLoRa.CS_pin          = NSS_Pin;
	  myLoRa.reset_port      = RST_GPIO_Port;
	  myLoRa.reset_pin       = RST_Pin;
	  myLoRa.DIO0_port       = DIO0_GPIO_Port;
	  myLoRa.DIO0_pin        = DIO0_Pin;
	  myLoRa.hSPIx           = &hspi1;
	  myLoRa.frequency             = 433       ;
	  myLoRa.spredingFactor        = SF_7      ;
	  myLoRa.bandWidth			   = BW_125KHz ;
	  myLoRa.crcRate               = CR_4_5    ;
	  myLoRa.power				   = POWER_20db;
	  myLoRa.overCurrentProtection = 100       ;
	  myLoRa.preamble			   = 8         ;
}
void configureGPRS(void)
{
  char response[50];

  sendATCommand("AT+CGATT=1\r\n", response);
  HAL_Delay(1000);

  sendATCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", response);
  HAL_Delay(1000);

  char apnCommand[50];
  sprintf(apnCommand, "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", APN);
  sendATCommand(apnCommand, response);
  HAL_Delay(1000);

  sendATCommand("AT+SAPBR=1,1\r\n", response);
  HAL_Delay(1000);

  sendATCommand("AT+SAPBR=2,1\r\n", response);
  HAL_Delay(1000);
}

void sendDataToThingSpeak(float tem, float hum, int i)
{
  char dataString[50];
  sprintf(dataString, "field%d=%.2f&field%d=%.2f",i,tem,i+1,hum);

  char url[100];
  strcpy(url, URL);
  strcat(url, dataString);

  char response[100];
  char dataLength[10];
  sprintf(dataLength, "%d", strlen(dataString));

  sendATCommand("AT+HTTPINIT\r\n", response);
  HAL_Delay(1000);

  sendATCommand("AT+HTTPPARA=\"CID\",1\r\n", response);
  HAL_Delay(1000);

  char urlCommand[150];
  sprintf(urlCommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
  sendATCommand(urlCommand, response);
  HAL_Delay(1000);

  char dataCommand[50];
  sprintf(dataCommand, "AT+HTTPDATA=%s,5000\r\n", dataLength);
  sendATCommand(dataCommand, response);
  HAL_Delay(1000);

  sendATCommand(dataString, response);
  HAL_Delay(1000);

  sendATCommand("AT+HTTPACTION=1\r\n", response);
  HAL_Delay(5000); // wait for HTTP response
  sendATCommand("AT+HTTPREAD\r\n", response);
}

void sendATCommand(char* command, char* response)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)command, strlen(command), 1000);
  HAL_UART_Receive(&huart2, (uint8_t*)response, 50, 1000);
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
