/*****安信可EC_01G多模定位+NBIot模组******
 * Ver0.1.0
 *
 * 2021/12/18
 * https://www.github.com/Cube-Line
 *
 *
 *
 * */
#include "stdlib.h"
#include "usart.h"
#include "ec01g_drv.h"

char uart_data[] = "AT+CGSN=0\n";

void EC_Power_Control(uint8_t EC_PWR)
{
	if (0x01 == EC_PWR)
		HAL_GPIO_WritePin(EC_PWR_GPIO_Port, EC_PWR_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(EC_PWR_GPIO_Port, EC_PWR_Pin, GPIO_PIN_RESET);
}

void EC_Init(void)
{
	EC_Power_Control(0x01); //打开POWER_KEY(EC_RST);
	HAL_UART_Transmit(&huart1, (uint8_t *)&uart_data, sizeof(uart_data), 1000);
}