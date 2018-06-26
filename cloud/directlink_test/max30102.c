


#include "max30102.h"


void MAX_i2c_init(void)
{
	

	Ql_IIC_Init(1,PINNAME_RI,PINNAME_DCD,true);
	
	Ql_IIC_Config(1,TRUE, I2C_WRITE_ADDR, 300);

	Ql_IIC_Config(1,TRUE, I2C_READ_ADDR, 300);
	
//
//	if(ret1 == 	 QL_RET_OK){
//   		mprintf("pins(SCL=%d,SDA=%d) IIC controller Ql_IIC_Init channel 1 success ret=%d\r\n",PINNAME_RI,PINNAME_DCD,ret1);
//    }else if(ret1 == QL_RET_ERR_NOSUPPORTPIN){
//    	mprintf("\r\nFailed!! IIC controller Ql_IIC_Init channel 1 the input pin is invalid\r\n");
//	}else if(ret1 == QL_RET_ERR_PINALREADYSUBCRIBE){
//		mprintf("\r\nFailed!! IIC controller Ql_IIC_Init channel 1 the input pin is in use in other places\r\n");
//	}else if(ret1 == QL_RET_ERR_I2CHWFAILED){
//		mprintf("\r\nFailed!! IIC controller Ql_IIC_Init channel 1 hardware issue\r\n");
//	}else{
//		mprintf("unknown issue just can't init!");
//	}
//
//
//
//	if(ret3 == QL_RET_OK)
//	{
//		mprintf("IIC controller Ql_IIC_Config channel 1 config success ret=%d\r\n",ret3);
//	}else{
//		mprintf("IIC controller Ql_IIC_Config channel 1 fail ret=%d\r\n",ret3);
//	}

}

bool maxim_max30102_write_reg(u8 uch_addr, u8 uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{

    u8 ach_i2c_data[2];
    ach_i2c_data[0]=uch_addr;
    ach_i2c_data[1]=uch_data;

	if(Ql_IIC_Write(1, I2C_WRITE_ADDR, ach_i2c_data, sizeof(ach_i2c_data)) > 0)
        return true;
    else
        return false;
}

bool maxim_max30102_read_reg(u8 uch_addr, u8 *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{


  u8 ch_i2c_data;
  ch_i2c_data=uch_addr;
  
  if(Ql_IIC_Write(1, I2C_WRITE_ADDR, &ch_i2c_data, sizeof(ch_i2c_data)) < 0)
     return false;
  if(Ql_IIC_Read(1, I2C_READ_ADDR, &ch_i2c_data, sizeof(ch_i2c_data)) > 0)
  {
     *puch_data=ch_i2c_data;
     return true;
  }
  else
     return false;
}




bool MAX_Init()
{ 
  if(maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)==0) // INTR setting
 	return false;
  if(maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00)==0)
    return false;
  if(maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00)==0)  //FIFO_WR_PTR[4:0]
    return false;
  if(maxim_max30102_write_reg(REG_OVF_COUNTER,0x00)==0)  //OVF_COUNTER[4:0]
    return false;
  if(maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00)==0)  //FIFO_RD_PTR[4:0]
    return false;
  if(maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f)==0)  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return false;
  if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x03)==0)   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return false;
  if(maxim_max30102_write_reg(REG_SPO2_CONFIG,0x47)==0)  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return false;

  if(maxim_max30102_write_reg(REG_LED1_PA,0x3f)==0)   //Choose value for ~ 7mA for LED1
    return false;
  if(maxim_max30102_write_reg(REG_LED2_PA,0x3f)==0)   // Choose value for ~ 7mA for LED2
    return false;
  if(maxim_max30102_write_reg(REG_PILOT_PA,0x7f)==0)   // Choose value for ~ 25mA for Pilot LED
    return false;
  return true;
}

u8 maxim_max30102_read_fifo(u32 *pun_red_led, u32 *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
  u32 un_temp;
  u8 uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  u8 ach_i2c_data[6];
  
  //read and clear status register
  if(maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp)== false){
  	mprintf("read reg1 failed!");
  	}
  if(maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp)== false){
  	mprintf("read reg2 failed!");
  	}
  
  ach_i2c_data[0]=REG_FIFO_DATA;
  
  if(Ql_IIC_Write(1, I2C_WRITE_ADDR, ach_i2c_data, 1) < 0)
    return 0;
  if(Ql_IIC_Read(1, I2C_READ_ADDR, ach_i2c_data, 6) < 0)
    return 0;
 

  un_temp=(u8) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(u8) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(u8) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(u8) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(u8) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(u8) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  
  
  return 1;
}

u8 maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x40)==0)
        return 0;
    else
        return 1;    
}


