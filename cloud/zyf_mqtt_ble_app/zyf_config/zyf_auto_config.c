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
*0501:Ѫ�����Ͷ�
*05EA:����
*1792:LBS(γ��)
*087B:LBS(����)
*1480:GPS(γ��)
*00EA:GPS(����)
*023B:�豸��ѹ

*1E9D:�����ٷֱ�
*023A:�ź�ǿ��
*2557:����ʱ��
*66AA:LED��
*22BC:������
*/
u8 *AutomaticRD[READONLYNUM]={"023B","1E9D","023A","2557","1792","087B","1480","00EA","0501","05EA"};
u8 *AutomaticWR[READWRITE]={"66AA","22BC"};
