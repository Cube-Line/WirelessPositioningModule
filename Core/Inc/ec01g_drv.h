/*****安信可EC_01G多模定位+NBIot模组******
 * Ver0.1.0
 *
 * 2021/12/18
 * https://www.github.com/Cube-Line
 *
 *
 *
 * */
#ifndef EC01G_DRV_H
#define EC01G_DRV_H

#include "stdlib.h"
#include "stdint.h"
#include "stm32f0xx_hal.h"

// #define EC_POWER 
void EC_Init(void);
void EC_Uart_Send(uint8_t *data);
void EC_Uart_Received(void);

void EC_Power_Control(uint8_t);
#endif