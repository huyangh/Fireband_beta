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
 *     Set "C_PREDEF=-D __DIRECT_LINK__; CLOUD_SOLUTION =DIRECTLINK_TEST_SOLUTION" in gcc_makefile file. And compile the 
 *     app using "make clean/new".
 *     Download image bin to module to run.
 *
 * Author:
 * -------
 * 	huyangh

 		This program provide direct communication between W203CS and MAX30102 through i2c.
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * ver 0.1:  initialization OK
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




void proc_main_task(s32 taskId)
{
	u32 aun_ir_buffer[500]; //IR LED sensor data
	s32 n_ir_buffer_length;    //data length
	u32 aun_red_buffer[500];	//Red LED sensor data
	s32 n_sp02; //SPO2 value
	s8 ch_spo2_valid;	//indicator to show if the SP02 calculation is valid
	s32 n_heart_rate;	//heart rate value
	s8	ch_hr_valid;	//indicator to show if the heart rate calculation is valid
	u8 uch_dummy;
	

	
    ST_MSG msg;
	

	// Register & open UART port
    Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(UART_PORT1, 115200, FC_NONE);

   
	// GPIO initialization
	Ql_GPIO_Init(PINNAME_DTR, PINDIRECTION_IN, PINLEVEL_LOW, PINPULLSEL_PULLUP);
	mprintf("<-- Initialize GPIO pin (PINNAME_STATUS): output, high level, pull down -->\r\n");

	// IIC initialization
	MAX_i2c_init();

    if(maxim_max30102_reset() == 1){
   		 mprintf("reset success!\r\n");
    }else{
    	mprintf("reset failed!\r\n");
    }
    if(maxim_max30102_read_reg(0,&uch_dummy) == true){
   		 mprintf("read and clear success!\r\n");
	}else{
		mprintf("read and clear failed!\r\n");
	}//read and clear status register


	// MAX30102 initialization
    if(MAX_Init() == true){
		 mprintf("initialization of MAX success!\r\n");
    }
   
   


	 n_ir_buffer_length=500;//buffer length of 100 stores 5 seconds of samples running at 100sps
	   
	  s32 m,ret;
	  
	  //read the first 500 samples, and determine the signal range
	  for(m=0;m<n_ir_buffer_length;m++)
	  {
		  while(Ql_GPIO_GetLevel(PINNAME_DTR)==1);   //wait until the interrupt pin asserts
		  
		  ret = maxim_max30102_read_fifo((aun_red_buffer+m), (aun_ir_buffer+m));
		  if(ret == 0){
		  	mprintf("read fifo failed!\n");
		  	}
		  
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

			 
	       

		  }
		  
		  //take 100 sets of samples before calculating the heart rate.
		  for(m=400;m<500;m++)
		  {
		  	  
			  while(Ql_GPIO_GetLevel(PINNAME_DTR)==1);   //wait until the interrupt pin asserts
			  maxim_max30102_read_fifo((aun_red_buffer+m), (aun_ir_buffer+m)); //read from MAX30102 FIFO
		  
			  //send samples and calculation result to terminal program through UART
			  mprintf("HR=%d, ", n_heart_rate); 
			  mprintf("HRvalid=%d, ", ch_hr_valid);
			  mprintf("SpO2=%d, ", n_sp02);
			  mprintf("SPO2Valid=%d\r\n", ch_spo2_valid);
//			  Ql_Sleep(500);
		  }
		  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
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


	   

   void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
   {
   		}






