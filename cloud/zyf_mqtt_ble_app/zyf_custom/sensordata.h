#include "ql_trace.h"
#include "ql_system.h"
#include "ql_gpio.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "max30102.h"
#include "algorithm.h"
#include "uart.h"

#include "ql_timer.h"
#include "user_mqtt.h"

//#ifndef __SENSOR_DATA__
//#define __SENSOR_DATA__
//


typedef struct
{
	s32 hrdata;			//心率数据
	s32 spo2data;		//血氧数据
} Sensor_Data;

Sensor_Data sensor_data;



void StartMAX();
void CloseMAX();

void calcmax(s32 *phr, s32 *pspo2);


void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);


//#endif




