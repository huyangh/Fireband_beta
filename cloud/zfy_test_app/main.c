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

#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   512
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





//static u8 m_Read_Buffer[1024];
//static u8 m_buffer[100];

u32 aun_ir_buffer[500]; //IR LED sensor data
s32 n_ir_buffer_length;    //data length
u32 aun_red_buffer[500];    //Red LED sensor data
s32 n_sp02; //SPO2 value
s8 ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
s32 n_heart_rate;   //heart rate value
s8  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
u8 uch_dummy;


static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);

#define FAST_REGISTER
Enum_PinName pinname = PINNAME_DTR;
/*****************************************************************
* callback function
******************************************************************/
static void callback_eint_handle(Enum_PinName eintPinName, Enum_PinLevel pinLevel, void* customParam);

void proc_main_task(s32 taskId)
{
    s32 ret;
    ST_MSG msg;

    // Register & open UART port
    Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(UART_PORT1, 115200, FC_NONE);

    Ql_UART_Register(UART_PORT2, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(UART_PORT2, 115200, FC_NONE);

    APP_DEBUG("\r\n<--OpenCPU: IIC TEST!-->\r\n"); 

	APP_DEBUG("<--OpenCPU: eint.-->\r\n"); 
    #ifdef FAST_REGISTER
		/*************************************************************
		* Registers an EINT I/O, and specify the interrupt handler. 
		*The EINT, that is registered by calling this function, is a lower-level interrupt. 
		*The response for interrupt request is more timely.
		*Please don't add any task schedule in the interrupt handler.
		*And the interrupt handler cannot consume much CPU time.
		*Or it causes system exception or reset.
		**************************************************************/
		ret = Ql_EINT_RegisterFast(pinname,callback_eint_handle, NULL);
    #else
		//Registers an EINT I/O, and specify the interrupt handler. 
		ret = Ql_EINT_Register(pinname,callback_eint_handle, NULL);    
    #endif
		if(ret != 0)
		{
			APP_DEBUG("<--OpenCPU: Ql_EINT_RegisterFast fail.-->\r\n"); 
			return;    
		}
		APP_DEBUG("<--OpenCPU: Ql_EINT_RegisterFast OK.-->\r\n"); 

		ret = Ql_EINT_Init(pinname, EINT_LEVEL_TRIGGERED, 0, 5,0);
		   if(ret != 0)
		   {
			   APP_DEBUG("<--OpenCPU: Ql_EINT_Init fail.-->\r\n"); 
			   return;	  
		   }
		   APP_DEBUG("<--OpenCPU: Ql_EINT_Init OK.-->\r\n");




		
	   MAX_i2c_init();

	   if(maxim_max30102_reset()==1){		 //resets the MAX30102
	   		APP_DEBUG("reset of max30102 success!");
	   }
	   else{
	   		APP_DEBUG("reset of max30102 failed!");
	   }
	   maxim_max30102_read_reg(0,&uch_dummy);	 //read and clear status register
	  
	   if(MAX_Init()==1){//initializes the MAX30102
	   		APP_DEBUG("initialize of max30102 success!");
	   }
	   else{
	     	APP_DEBUG("initialize of max30102 failed!");
	   }
	   
	   n_ir_buffer_length=500;//buffer length of 100 stores 5 seconds of samples running at 100sps

	   s32 m;
	   //read the first 500 samples, and determine the signal range
	   for(m=0;m<n_ir_buffer_length;m++)
	   {
		   while(ret==0);   //wait until the interrupt pin asserts
		   
		   if(maxim_max30102_read_fifo((aun_red_buffer+m), (aun_ir_buffer+m))==1){
		   		APP_DEBUG("read fifo success!");
		   }else{
		   		APP_DEBUG("read fifo failed!");
		   }//read from MAX30102 FIFO
			   
	   }
	   //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
	   maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	
	   //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
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
			   while(ret==0);     //wait until the interrupt pin asserts
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



	   static void callback_eint_handle(Enum_PinName eintPinName, Enum_PinLevel pinLevel, void* customParam)
	   {
		   s32 ret;
		   //mask the specified EINT pin.
		   Ql_EINT_Mask(pinname);
		   
		   APP_DEBUG("<--Eint callback: pin(%d), level(%d)-->\r\n",eintPinName,pinLevel);
		   ret = Ql_EINT_GetLevel(eintPinName);
		   APP_DEBUG("<--Get Level, pin(%d), level(%d)-->\r\n",eintPinName,ret);
		   
		   //unmask the specified EINT pin
		   Ql_EINT_Unmask(pinname);
	   }

	   static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
	   {
	   	}




/*
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
    u32 rdLen=0;
    s32 ret;
    char *p=NULL;
    char *p1=NULL;
    char *p2=NULL;
    u8 write_buffer[4]={'a','b','c','D'};
    u8 read_buffer[6]={0x00,0x00,0x00,0x00,0x00,0x00};
    u8 registerAdrr[2]={0x01,0x45};
    switch (msg)
    {
        case EVENT_UART_READY_TO_READ:
        {
            Ql_memset(m_Read_Buffer, 0x0, sizeof(m_Read_Buffer));
            rdLen = Ql_UART_Read(port, m_Read_Buffer, sizeof(m_Read_Buffer));

            //command-->Init the IIC , type 0 Лђеп 1.
            p = Ql_strstr(m_Read_Buffer,"Ql_IIC_Init=");
            if(p)
            {
                char* p1 = NULL;
                char* p2 = NULL;
                u8 NumberBuf[10];
                s8 IIC_type=0;
                p1 = Ql_strstr(m_Read_Buffer, "=");
                p2 = Ql_strstr(m_Read_Buffer, "\r\n");
                Ql_memset(NumberBuf, 0x0, sizeof(NumberBuf));
                Ql_memcpy(NumberBuf, p1 + 1, p2 - p1 -1);
                IIC_type = Ql_atoi(NumberBuf);
                if(0 == IIC_type)// simultion iic test, and we choose PINNAME_GPIO4, PINNAME_GPIO5 for IIC SCL and SDA pins
                {                    
                    ret = Ql_IIC_Init(0,PINNAME_CTS,PINNAME_RTS,0);
                    if(ret < 0)
                    {
                        APP_DEBUG("\r\n<--Failed!! Ql_IIC_Init channel 0 fail ret=%d-->\r\n",ret);
                        break;
                    }
                    APP_DEBUG("\r\n<--pins(%d & %d) Ql_IIC_Init channel 0 ret=%d-->\r\n",PINNAME_CTS,PINNAME_RTS,ret);
                    break;
                }
                else if(1 == IIC_type)// IIC controller
                {
                    ret = Ql_IIC_Init(1,PINNAME_RI,PINNAME_DCD,1);
                    if(ret < 0)
                    {
                        APP_DEBUG("\r\n<--Failed!! IIC controller Ql_IIC_Init channel 1 fail ret=%d-->\r\n",ret);
                        break;
                    }
                    APP_DEBUG("\r\n<--pins(SCL=%d,SDA=%d) IIC controller Ql_IIC_Init channel 1 ret=%d-->\r\n",PINNAME_RI,PINNAME_DCD,ret);
                    break;

                }
                else
                {
                    APP_DEBUG("\r\n<--IIC type error!!!!-->\r\n");
                    break;
                }

            }
	     //command-->IIC config,  IIC controller interface
            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Config=0\r\n");//   simultion IIC  (channel 0)
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Config(0,TRUE, 0x07, 0);// simultion IIC interface ,do not care the IicSpeed.
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !!Ql_IIC_Config channel 0 fail ret=%d-->\r\n",ret);
                    break;
                }
                APP_DEBUG("\r\n<--Ql_IIC_Config channel 0 ret=%d-->\r\n",ret);
                break;
            }
            
            //command-->IIC config,  IIC controller interface
            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Config=1\r\n");//   IIC controller  (channel 1)
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Config(1,TRUE, 0x07, 300);// just for the IIC controller
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! IIC controller Ql_IIC_Config channel 1 fail ret=%d-->\r\n",ret);
                    break;
                }
                APP_DEBUG("\r\n<--IIC controller Ql_IIC_Config channel 1 ret=%d-->\r\n",ret);
                break;
            }
            
            //command-->IIC write  
            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Write=0\r\n");//  channel 0 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Write(0, 0x07, write_buffer, 4);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! Ql_IIC_Write channel 0 fail ret=%d-->\r\n",ret);
                    break;               
                }
                APP_DEBUG("\r\n<--channel 0 Ql_IIC_Write ret=%d-->\r\n",ret);
                break;
            }

            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Write=1\r\n");//  channel 1 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Write(1, 0x07, write_buffer,4);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! IIC controller Ql_IIC_Write channel 1 fail ret=%d-->\r\n",ret);
                    break;
                }
                APP_DEBUG("\r\n<--IIC controller Ql_IIC_Write ret=%d-->\r\n",ret);
                break;
            }

            //command-->IIC read  
            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Read=0\r\n");//  channel 0 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Read(0, 0x07, read_buffer, 6);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! Ql_IIC_Read channel 0 fail ret=%d-->\r\n",ret);
                    break;               
                }
                APP_DEBUG("\r\n<--channel 0 Ql_IIC_Read ret=%d-->\r\n",ret);
                break;
            }

            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Read=1\r\n");//  channel 1 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Read(1, 0x07, write_buffer, 6);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! IIC controller Ql_IIC_Read channel 1 fail ret=%d-->\r\n",ret);
                    break;
                }
                APP_DEBUG("\r\n<--IIC controller Ql_IIC_Read ret=%d-->\r\n",ret);
                break;
            }

            //command-->IIC write then read  
            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Write_Read=0\r\n");//  channel 0 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Write_Read(0, 0x07, registerAdrr, 2,read_buffer, 6);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! Ql_IIC_Write_Read channel 0 fail ret=%d-->\r\n",ret);
                    break;               
                }
                APP_DEBUG("\r\n<--channel 0 Ql_IIC_Write_Read ret=%d-->\r\n",ret);
                break;
            }

            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Write_Read=1\r\n");//  channel 1 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Write_Read(1, 0x07, registerAdrr, 2,read_buffer, 6);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! IIC controller Ql_IIC_Write_Read channel 1 fail ret=%d-->\r\n",ret);
                    break;
                }
                APP_DEBUG("\r\n<--IIC controller Ql_IIC_Write_Read ret=%d-->\r\n",ret);
                break;
            }

                
            
            //command-->IIC write then read  
            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Uninit=0\r\n");//  channel 0 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Uninit(0);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! Ql_IIC_Uninit channel 0 fail ret=%d-->\r\n",ret);
                    break;               
                }
                APP_DEBUG("\r\n<--channel 0 Ql_IIC_Uninit ret=%d-->\r\n",ret);
                break;
            }

            Ql_memset(m_buffer, 0x0, sizeof(m_buffer));
            Ql_sprintf(m_buffer, "Ql_IIC_Uninit=1\r\n");//  channel 1 
            ret = Ql_strncmp(m_Read_Buffer, m_buffer, Ql_strlen(m_buffer));                
            if(0 == ret)
            {
                ret = Ql_IIC_Uninit(1);
                if(ret < 0)
                {
                    APP_DEBUG("\r\n<--Failed !! IIC controller Ql_IIC_Uninit channel 1 fail ret=%d-->\r\n",ret);
                    break;
                }
                 APP_DEBUG("\r\n<--IIC controller (chnnlNo 1) Ql_IIC_Uninit  ret=%d-->\r\n",ret);
                break;
            }
            
        APP_DEBUG("\r\n<--Not found this command, please check you command-->\r\n");
        }
    default:
        break;
    }
}
*/

