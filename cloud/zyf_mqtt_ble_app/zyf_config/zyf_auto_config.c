#include "zyf_auto_config.h"
ZYF_AUTO_CONFIG_T ZYFAutoconfig=
{
    "40b1a69ea20d",
	 1883,
	 "www.hizyf.com",
	 "652541",
	 "admin",
	 "zyfadmin",
	 "",
};
/**
*0501:血氧饱和度
*05EA:心率
*1792:LBS(纬度)
*087B:LBS(经度)
*1480:GPS(纬度)
*00EA:GPS(经度)
*023B:设备电压

*1E9D:电量百分比
*023A:信号强度
*2557:运行时长
*66AA:LED灯
*22BC:传输间隔
*/
u8 *AutomaticRD[READONLYNUM]={"023B","1E9D","023A","2557","1792","087B","1480","00EA","0501","05EA"};
u8 *AutomaticWR[READWRITE]={"66AA","22BC"};
