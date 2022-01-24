/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "ec01g_drv.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// extern ec_module ec01g;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// uint8_t aRxBuffer[10];
uint8_t aTxStartMessage[] = "\n\r ****UART-Hyperterminal communication based on DMA****\n\r Enter 10 characters using keyboard :\n\r";
uint8_t EC_RESET[] = "AT+ECRST\n";																									  //复位EC-01G;
uint8_t EC_CLOSE_FLY_MODE[] = "AT+CFUN=1\n";																						  //关闭飞行模式;
uint8_t EC_CEREG[] = "AT+CEREG?";																									  //判断PS域附着状态，第二个参数为1或5表示附着正常;
uint8_t EC_SELT_PLATFORM[] = "AT+ECMTCFG=\" cloud\",0,2,1\n";																		  //设置平台为阿里云物联网，数据类型为string格式;
uint8_t EC_SET_CERTIFICATE[] = "AT+ECMTCFG=\"aliauth\",0,\"a1VnGy2Lv50\",\"863155050030517\",\"038d0cddce7d402307b898f061b72bee\"\n"; //分别写入刚才我们生成测试设备的设备证书的ProductKey、DeviceName、DeviceSecret;
uint8_t EC_CREATE_TCP[] = "AT+ECMTOPEN=0,\"a1VnGy2Lv50.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883\n";								  //建立 tcp，其中域名组成方式为${YourProductKey}.iot-as-mqtt.${YourRegionId}.aliyuncs.com，${YourProductKey}即设备ProductKey，${YourRegionId}即地域，这里我们可以统一选择cn-shanghai，端口统一为1883;
uint8_t EC_CREATE_MQTT[] = "AT+ECMTCONN=0, \"12345\"\n";																			  //创建 mqtt，在阿里物联网平台上注册设备，clientID可为任意字符串（最大48个字节）
uint8_t aRxBuffer[RXBUFFERSIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	// OLED_Init();
	//   OLED_WR_Byte(0x55, 0x00);
	HAL_GPIO_WritePin(LED_LINK_GPIO_Port, LED_LINK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_NET_GPIO_Port, LED_NET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GPS_GPIO_Port, LED_GPS_Pin, GPIO_PIN_SET);
	// OLED_Clear();
	// OLED_DrawPoint(16, 16, 1);
	// OLED_ShowNum(0, 0, 88, 2, 16);
	// OLED_ShowString(16, 0, "B");
	// OLED_Fill(0, 0, 127, 63, 1);
	// OLED_Refresh_Gram(); //更新显示
	//  OLED_Display_On();

	EC_Init();
	//复位EC模块;
	if (HAL_OK != HAL_UART_Transmit_DMA(&huart1, (uint8_t *)EC_RESET, COUNTOF(EC_RESET) - 1))
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}
	//关闭飞行模式;
	if (HAL_OK != HAL_UART_Transmit_DMA(&huart1, (uint8_t *)EC_CLOSE_FLY_MODE, COUNTOF(EC_CLOSE_FLY_MODE) - 1))
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}

	if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	{
		/* Transfer error in reception process */
		Error_Handler();
	}

	while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
	{
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_GPIO_TogglePin(LED_LINK_GPIO_Port, LED_LINK_Pin);
		HAL_Delay(100);
		// OLED_Power_Control(0);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
