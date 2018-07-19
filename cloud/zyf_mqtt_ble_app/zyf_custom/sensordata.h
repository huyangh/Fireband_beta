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


typedef struct{
	s32  hrdata;
	s32  spo2data;
}SENSOR_DATA;

SENSOR_DATA	_sensor_data_new;

void StartMAX();

void calcmax(u8 calc);

void Getsensordata(SENSOR_DATA sensordata, s32 hr, s32 spo2,s8 hrvalid, s8 spo2valid);

void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);




