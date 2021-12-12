#ifndef __OLED_H
#define __OLED_H
// #include "sys.h"
#include "stdlib.h"
#include "stdint.h"
#include "stm32f0xx_hal.h"

//---------------------------OLED端口定义--------------------------

//使用4线串行接口时使用
#define OLED_SCLK I2C1_SCL
#define OLED_SDIN I2C1_SDA

#define OLED_ADDRESS 0X0078

//OLED控制用函数
void OLED_Power_Control(uint8_t pwr_sw);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);

void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p);


#endif
