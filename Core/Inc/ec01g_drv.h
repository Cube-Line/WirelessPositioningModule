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

// struct ec_module
// {
// 	char *at_test;				  // AT测试;
// 	char *reset_module;			  //复位EC-01G;
// 	char *close_flight_mode;	  //关闭飞行模式;
// 	char *get_iccid;			  //查询SIM卡ICCID号;
// 	char *attached_network;		  //附着网络;
// 	char *register_APNID;		  //注册APNID;
// 	char *active_network;		  //激活网络;
// 	char *test_net_status;		  // ping网络,测试网络状态;
// 	char *connect_tcp_web_server; //使用TCP协议连接到服务器;
// 	char *connect_udp_web_server; //使用UDP协议连接到服务器;
// 	char *start_connect;		  //开始连接
// 	char *act_gps;				  //开启GPS;
// 	char *get_iemi;				  //获取设备IEMI号;
// 	char *get_singnal_strength;	  //获取信号质量;
// } ec01g = {
// 	"AT",								   // AT测试;
// 	"AT+ECRST\n",						   //复位EC-01G;
// 	"AT+CFUN=1\n",						   //关闭飞行模式;
// 	"AT+ECICCID=1",						   //查询SIM卡ICCID号;
// 	"AT+CGATT=1",						   //附着网络;
// 	"AT + CGDCONT = 1, \"IP\", \"CMNET\"", //注册APNID;
// 	"AT+CGACT=1",						   //激活网络;
// 	"AT+ECPING=\"www.baidu.com\"",		   // ping网络,测试网络状态;
// 	"AT+SKTCREATE=1,1,6",				   //使用TCP协议连接到服务器;
// 	"AT+SKTCREATE=1,2,17",				   //使用UDP协议连接到服务器;
// 	"AT+SKTCONNECT=",					   //开始连接
// 	"AT+GPS=1\n",						   //开启GPS;
// 	"AT+CGSN=1\n",						   //获取设备IEMI号;
// 	"AT+CSQ\n",							   //获取信号质量;
// };

// #define EC_POWER
void EC_Init(void);
void EC_Uart_Send(uint8_t *data);
void EC_Uart_Received(void);

void EC_Power_Control(uint8_t);
#endif
