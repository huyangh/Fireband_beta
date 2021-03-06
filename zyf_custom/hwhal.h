#ifndef _HWHAAL_H
#define _HWHAAL_H

#include "ql_type.h"
#include "ql_trace.h"
#include "ql_gpio.h"


#define LED_R_ON	Ql_GPIO_SetLevel(PINNAME_SD_CMD, PINLEVEL_HIGH)
#define LED_R_OFF	Ql_GPIO_SetLevel(PINNAME_SD_CMD, PINLEVEL_LOW)
#define LED_G_ON	Ql_GPIO_SetLevel(PINNAME_SD_DATA, PINLEVEL_HIGH)
#define LED_G_OFF	Ql_GPIO_SetLevel(PINNAME_SD_DATA, PINLEVEL_LOW)
#define LED_B_ON	Ql_GPIO_SetLevel(PINNAME_NETLIGHT, PINLEVEL_HIGH)
#define LED_B_OFF	Ql_GPIO_SetLevel(PINNAME_NETLIGHT, PINLEVEL_LOW)
#define BB_ON		Ql_GPIO_SetLevel(PINNAME_SD_CLK, PINLEVEL_HIGH)
#define BB_OFF		Ql_GPIO_SetLevel(PINNAME_SD_CLK, PINLEVEL_LOW)
#define M203C_DATA_ON		Ql_GPIO_SetLevel(PINNAME_CTS, PINLEVEL_HIGH)
#define M203C_DATA_OFF		Ql_GPIO_SetLevel(PINNAME_CTS, PINLEVEL_LOW)
#define SPK_AMP_EN_ON		Ql_GPIO_SetLevel(PINNAME_DCD, PINLEVEL_HIGH)
#define SPK_AMP_EN_OFF		Ql_GPIO_SetLevel(PINNAME_DCD, PINLEVEL_LOW)


void hwhalinit(void);
#endif
