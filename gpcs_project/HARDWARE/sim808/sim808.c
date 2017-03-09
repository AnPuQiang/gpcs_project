#include "sim808.h"
#include "usart.h"
#include "delay.h"	
//#include "led.h"   	 
//#include "key.h"	 	 	 	 	 
//#include "flash.h" 	 	 
//#include "malloc.h"
#include "string.h"    	
#include "usart2.h" 
#include "usart.h"
//#include "ff.h"

extern vu8 Timer0_start;
extern u8 shijian;
extern u8 Times;

//�����յ�������ظ���strΪ��ȷ�Ļظ�
//�ظ����󷵻�0���ظ���ȷ����str��λ��
u8* sim808_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART2_RX_STA&0X8000)	//
	{
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str);
	}
	return (u8*)strx;
}
//�������ͨ������2���з���
//cmd�����͵������ַ���������Ҫ�س���cmd<0XFF�������֣��������ַ���
//����ֵ��0���ɹ���1��ʧ��
u8 sim808_send_cmd(u8 *cmd,u8 *ack,u16 wait_time)
{	
	u8 res=0;
	USART2_RX_STA=0;
	u2_printf("%s\r\n",cmd);
	if(ack&&wait_time)
	{
		while(--wait_time)
		{
			delay_ms(10);
			if(USART2_RX_STA&0X8000)
				{
			if(sim808_check_cmd(ack)) break;
				USART2_RX_STA=0;
				}
		}
		if(wait_time==0)res=1;
		Times = 0;
		shijian = wait_time;
		Timer0_start = 1;
	}
	return res;
	
}

void Wait_CREG(void)
{
	u8 i,k;
	i = 0;
	CLR_Buf2();
	while(0 == i)
	{
		CLR_Buf2();
		UART2_SENDString("AT+CREG?");
		UART2_SendRN();
		delay_ms(5000);
		for(k=0;k<USART2_MAX_RECV_LEN;k++)
		{
			if(':' == USART2_RX_BUF[k])
			{
				if((USART2_RX_BUF[k+4]=='1')||(USART2_RX_BUF[k+4]=='5'))
				{
					i=1;
					printf("\r\n");
					break;
				}
			}
		}
	printf("ע����.....");
	}
}

void Send_OK(void)
{
	sim808_send_cmd("AT+CIPSEND",">",2);
	sim808_send_cmd("OK1234\r\n\32\0","SEND OK",8);			//�ظ�OK 
}


