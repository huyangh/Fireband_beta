/*****************************************************************************
 *
 * Filename:
 * ---------
 *   main.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 *
 * Usage:
 * ------
 *   Compile & Run:
 *
 *     Set "C_PREDEF=-D __ZFY_TEST_APP__; CLOUD_SOLUTION =ZYF_TEST_SOLUTION " in gcc_makefile file. And compile the 
 *     app using "make clean/new".
 *     Download image bin to module to run.
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/


#include "ql_trace.h"
#include "ql_system.h"
#include "ql_gpio.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "max30102.h"
#include "algorithm.h"
#include "ql_eint.h"
#include "uart.h"

#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   512

//#define FAST_REGISTER
Enum_PinName pinname = PINNAME_DTR;

static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...) 
#endif

//#ifdef FAST_REGISTER
//		/*************************************************************
//		* Registers an EINT I/O, and specify the interrupt handler. 
//		*The EINT, that is registered by calling this function, is a lower-level interrupt. 
//		*The response for interrupt request is more timely.
//		*Please don't add any task schedule in the interrupt handler.
//		*And the interrupt handler cannot consume much CPU time.
//		*Or it causes system exception or reset.
//		**************************************************************/
//		ret = Ql_EINT_RegisterFast(pinname,callback_eint_handle, NULL);
//#else
//		//Registers an EINT I/O, and specify the interrupt handler. 
//		ret = Ql_EINT_Register(pinname,callback_eint_handle, NULL);    
//#endif






u32 aun_ir_buffer[500]; //IR LED sensor data
s32 n_ir_buffer_length;    //data length
u32 aun_red_buffer[500];    //Red LED sensor data
s32 n_sp02; //SPO2 value
s8 ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
s32 n_heart_rate;   //heart rate value
s8  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
u8 uch_dummy;




void proc_main_task(s32 taskId)
{
    s32 ret;
    ST_MSG msg;

    // Register & open UART port
    Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(UART_PORT1, 115200, FC_NONE);

    Ql_UART_Register(UART_PORT2, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(UART_PORT2, 115200, FC_NONE);

    ret = Ql_EINT_RegisterFast(pinname,callback_eint_handle, NULL);
		if(ret != 0)
		{
			APP_DEBUG("<--OpenCPU: Ql_EINT_RegisterFast fail.-->\r\n"); 
			return;    
		}else{
		APP_DEBUG("<--OpenCPU: Ql_EINT_RegisterFast OK.-->\r\n"); 
		}

		ret = Ql_EINT_Init(pinname, EINT_LEVEL_TRIGGERED, 0, 5,0);
		   if(ret != 0)
		   {
			   APP_DEBUG("<--OpenCPU: Ql_EINT_Init fail.-->\r\n"); 
			   return;	  
		   }
		   APP_DEBUG("<--OpenCPU: Ql_EINT_Init OK.-->\r\n");

	   MAX_i2c_init();

	   if(maxim_max30102_reset()==1){		 //resets the MAX30102
	   		mprintf("reset of max30102 success!\n");
	   }
	   else{
	   		mprintf("reset of max30102 failed!\n");
	   }
	   maxim_max30102_read_reg(0,&uch_dummy);	 //read and clear status register
	  
	   if(MAX_Init()==1){//initializes the MAX30102
	   		mprintf("initialize of max30102 success!\n");
	   }
	   else{
	     	mprintf("initialize of max30102 failed!\n");
	   }
	   
	   

    while (1)
    {
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
        case 0:
            break;
        default:
            break;
        }
    }
}


	   void callback_eint_handle(Enum_PinName eintPinName, Enum_PinLevel pinLevel, void* customParam)
	   {
		   s32 ret;
		   
		   if(PINNAME_DTR==eintPinName)
		   {
		   	   ret = Ql_EINT_GetLevel(eintPinName);
			   Ql_EINT_Unmask(eintPinName);
		   	   APP_DEBUG("<--Eint callback: pin(%d), level(%d)-->\r\n",eintPinName,pinLevel);

			   n_ir_buffer_length=500;//buffer length of 100 stores 5 seconds of samples running at 100sps
		   
				  s32 m;
				  //read the first 500 samples, and determine the signal range
				  for(m=0;m<n_ir_buffer_length;m++)
				  {
					  while(ret==0);   //wait until the interrupt pin asserts
					  
					  if(maxim_max30102_read_fifo((aun_red_buffer+m), (aun_ir_buffer+m))==1){
						   mprintf("read fifo success!\n");
					  }else{
						   mprintf("read fifo failed!\n");
					  }//read from MAX30102 FIFO
						  
				  }
				  //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
				  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
			   
				  //Continuously taking samples from MAX30102.	Heart rate and SpO2 are calculated every 1 second
				  while(1)
				  {
					  m=0;
					  
					  //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
					  for(m=100;m<500;m++)
					  {
						  aun_red_buffer[m-100]=aun_red_buffer[m];
						  aun_ir_buffer[m-100]=aun_ir_buffer[m];
						  
					  //take 100 sets of samples before calculating the heart rate.
					  for(m=400;m<500;m++)
					  {
						  while(ret==0);	 //wait until the interrupt pin asserts
						  maxim_max30102_read_fifo((aun_red_buffer+m), (aun_ir_buffer+m)); //read from MAX30102 FIFO
					  
						  //send samples and calculation result to terminal program through UART
						  mprintf("HR=%d, ", n_heart_rate); 
						  mprintf("HRvalid=%c, ", ch_hr_valid);
						  mprintf("SpO2=%d, ", n_sp02);
						  mprintf("SPO2Valid=%c\r\n", ch_spo2_valid);
							   
					  }
					  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
					  }
				   }
		   	}
	   }

	   static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
	   {
	   	}






