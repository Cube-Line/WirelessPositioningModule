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
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "ec01g_drv.h"

// #define EC_RST



char *EC_NB_init_data[] = {
		"AT+ECRST\n",	//复位EC-01G;
		"AT+CFUN=1\n",	//关闭飞行模式;
		"AT+CGATT=1\n", //附着网络;
};

char *EC_create_mqtt[] = {
	"AT + ECMTCFG = \"cloud\", 0, 3, 0\n",				//创建MQTT客户端
	"AT +ECMTOPEN = 0,\"wx.ai-thinker.com\", 1883\n",	//打开客户端连接
	"AT +ECMTCONN = 0,\"123\"，\"admin\",\"public\"\n", //创建连接
};
char *EC_recivemessage_mqtt[] = {
	"AT+ECMTSUB=0,123,\"/IMEI/devPub\",0\n", //订阅数据;
};

char *EC_getlocation[] = {
	"AT + LOCATION = 1\n", //获取位置信息;
	"AT+GPSRD=1\n",		   //设置NMEA输出时间间隔;
};
char *EC_sendmessage_mqtt[] = {
	"AT+ECMTPUB=0,1234,0,1,\"imei/DEVpUB\",\"{\"LONG\":value,\"qos\":value,\"temper\":value,\"hum\":value}\n", //上传数据;
};

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
	HAL_UART_Transmit(&huart1, (uint8_t *)EC_NB_init_data, sizeof(&EC_NB_init_data), 1000);
}