#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "includes.h"
#include "exfuns.h"
#include "malloc.h"	 
#include "usart2.h"
#include "timer.h"
#include "sim808.h"
#include "mmc_sd.h"		
#include "ff.h"
/************************************************
************************************************/

//UCOSIII���������ȼ��û�������ʹ�ã�ALIENTEK
//����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
//���ȼ�0���жϷ������������� OS_IntQTask()
//���ȼ�1��ʱ�ӽ������� OS_TickTask()
//���ȼ�2����ʱ���� OS_TmrTask()
//���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
//���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()
//�������ȼ�
#define START_TASK_PRIO		8
//�����ջ��С	
#define START_STK_SIZE 		64
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
__align(8) static CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//�������ȼ�
#define TASK1_TASK_PRIO		7
//�����ջ��С	
#define TASK1_STK_SIZE 		64
//������ƿ�
OS_TCB Task1_TaskTCB;
//�����ջ	
__align(8) static CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
void task1_task(void *p_arg);


//�����Ϊ��������
//�������ȼ�
#define TASK2_TASK_PRIO		9
//�����ջ��С	
#define TASK2_STK_SIZE 		128
//������ƿ�
OS_TCB Task2_TaskTCB;
//�����ջ	
__align(8) static CPU_STK TASK2_TASK_STK[TASK2_STK_SIZE];
void task2_task(void *p_arg);

u8 share_resource[1];   //������Դ��

OS_SEM	MY_SEM;		//����һ���ź��������ڷ��ʹ�����Դ

FATFS fatfs;													/* FatFs�ļ�ϵͳ���� */
FIL fnew;													/* �ļ����� */
FRESULT res_sd;                								/* �ļ�������� */
uint16_t i;

const char *IP_command = "AT+CIPSTART=\"TCP\",\"123.206.41.146\",\"8083\"";
char *p1,*p2; 
char *time,*lati,*longi,*gps;
 
u8 Times=0,First_Int = 0,shijian=0;
u16 Heartbeat=0;
vu8 Heart_beat;//��������֡��־λ

vu8 Timer0_start;	//��ʱ��0��ʱ����������
u8 buffer[200];


u8 gps_analysis(void);
u8 NMEA_Comma_Pos(u8 *buf,u8 cx);
u32 NMEA_Pow(u8 m,u8 n);
void data_storage(void);
void connecting_server(void);

//������
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	u8 res=1;
	u16 temp=0;
	u32 dtsize,dfsize;	//SD��ʣ��������������
	delay_init();  //ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
	uart_init(115200);   //���ڳ�ʼ��

	Timer2_Init_Config();
	LED_Init();         //LED��ʼ��	
	KEY_Init();			//������ʼ��
	//SPI_Flash_Init();		//W25Qxx��ʼ��												  
  mem_init();				//�ڴ�س�ʼ��
	exfuns_init();
	f_mount(fs[0],"0:",1);	//����SD��
	while(SD_Initialize())	
	{
		printf("SD error\r\n");
	}
	do
	{
		temp++;
 		res=exf_getfree("0:",&dtsize,&dfsize);//�õ�SD��ʣ��������������
		delay_ms(200);		   
	}while(res&&temp<5);//�������5��
	if(res==0)
	{	
		printf("SD��OK ʣ��%dMB,�ܹ�%dMB\r\n",dtsize>>10,dfsize>>10);
	}
	else
  {
		printf("SD��error");
		
	}
	USART2_Init(115200);
	if(sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000))//��GPS��Դ!=0
	{printf("ʧ��%s\r\n",USART2_RX_BUF);}
	else{printf("�ɹ�%s\r\n",USART2_RX_BUF);}
	
			
	
	OSInit(&err);		    	//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();	//�����ٽ���			 
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);      //����UCOSIII
}


//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
		
	OS_CRITICAL_ENTER();	//�����ٽ���
	//����һ���ź���
	OSSemCreate ((OS_SEM*	)&MY_SEM,
                 (CPU_CHAR*	)"MY_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
	//����TASK1����
	OSTaskCreate((OS_TCB 	* )&Task1_TaskTCB,		
				 (CPU_CHAR	* )"Task1 task", 		
                 (OS_TASK_PTR )task1_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK1_TASK_PRIO,     
                 (CPU_STK   * )&TASK1_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK1_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK1_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);			
	//����TASK2����
	OSTaskCreate((OS_TCB 	* )&Task2_TaskTCB,		
				 (CPU_CHAR	* )"Task2 task", 		
                 (OS_TASK_PTR )task2_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK2_TASK_PRIO,     
                 (CPU_STK   * )&TASK2_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK2_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK2_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				 
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}

//����1�������� ��SD����д������
void task1_task(void *p_arg)
{
	OS_ERR err;
	u8 task1_str[]="!";
	char buf[100];
	char *buffa;
	u8 gps_able;
	u8 res_gprs;
	OSSemPend(&MY_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); 	//�����ź���
	memcpy(share_resource,task1_str,sizeof(task1_str)); //������Դ����������
//	USART2_Init(115200);
	connecting_server();
	res_gprs=sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000);	//��GPS��Դ
	printf("gps power:%d",res_gprs);
	while(1)
	{
		USART2_Init(115200);
		if(sim808_send_cmd("AT+CGNSINF\r\n","OK",1000)==0)
		{
			printf("�յ�GPS\r\n");
		}
//		Send_OK();
		delay_ms(500);
//��ý���֮�������
		gps_able=gps_analysis();
		if(gps_able)	//����ȡgps������ȷ
		{							//1�����ݴ洢2�����ݷ��Ͳ����
			data_storage();//gps���ݴ洢��SD����
			USART2_Init(115200);
			buffa=(char*)buffer;
			buffa = strcat((char*)buffa,"\r\n\32\0");
			res_gprs=sim808_send_cmd("AT+CIPSEND",">",200);
			printf("cipsend:%d\r\n",res_gprs);	//0�ɹ���1ʧ��
			res_gprs=sim808_send_cmd((u8*)buffa,"SEND OK",800);
			printf("send_ok:%d\r\n",res_gprs);
			delay_ms(500);
			memset(buffer,0,sizeof(buffer));	//���buffer
			if(Heart_beat)
			{
				Send_OK();
				sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000);
				Heart_beat=0;
			}
			
		}
//		printf("%s\r\n",share_resource);	//�������������Դ������	
		OSSemPost (&MY_SEM,OS_OPT_POST_1,&err);				//�����ź���
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err);   //��ʱ1s
	}
}

//����2��������
void task2_task(void *p_arg)
{	
	OS_ERR err;
//	u8 task2_str[]="!";
//	while(1)
//	{
//		printf("\r\n����2:\r\n");
//		OSSemPend(&MY_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); 	//�����ź���
//		memcpy(share_resource,task2_str,sizeof(task2_str));	//������Դ����������
//		delay_ms(300);
//		printf("%s\r\n",share_resource);	//�������������Դ������		
//		OSSemPost (&MY_SEM,OS_OPT_POST_1,&err);				//�����ź���
//		LED1 = ~LED1;
//		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err);   //��ʱ1s
//	}
}

u8 gps_analysis(void)	//����1֤���ɹ�
 {
	 u8 pos_begin;
	 u8 pos_end;
	 int i;
	 pos_begin = NMEA_Comma_Pos(USART2_RX_BUF,2);
   pos_end = NMEA_Comma_Pos(USART2_RX_BUF,5);
	 if(USART2_RX_BUF[pos_begin] == '1' || USART2_RX_BUF[pos_begin] == '2')	//�����ȡ����ݵĵ�һλ������2��֤��ʱ����ȷ
	 {
	 for(i = pos_begin;i<pos_end;i++)
	 {
		 buffer[i-pos_begin] = USART2_RX_BUF[i];
	 }
	 printf("USART2_RX_BUF:%s\r\n",USART2_RX_BUF);
   CLR_Buf2();
	 return 1;
 }
	 else{return 0;}
 }
 u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

//m^n����
//����ֵ:m^n�η�.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
void data_storage(void)
{
	u8 temp;
	u8 buffer_length;
	u8 file_name[13]={0};	//�ַ�������Ҫ��ʼ������
	char *char_filename;
	SD_Initialize();
//	printf("buffer:%s\r\n",buffer);
	for(temp=0;temp<8;temp++)
	{
		file_name[temp]=buffer[temp];
	}
	file_name[8]='.';file_name[9]='t';file_name[10]='x';file_name[11]='t';
	
	printf("file_name:%s\r\n",(char*)file_name);
	//����ȡ������������Ϣ��Ϊ�ļ���������txt�ļ�
	char_filename=(char*)file_name;
	printf("char_filename:%s\r\n",char_filename);
	//�򿪸��ļ�����bufferд���ļ���
	res_sd=f_open(file, char_filename, FA_OPEN_ALWAYS|FA_WRITE);
	printf("���ļ����ش��룺%d\r\n",res_sd);
	printf("file_size: %d\r\n",(int)(*file).fsize);
	f_lseek(file,(*file).fsize);
	buffer_length = strlen((char*)buffer);
	printf("buffer_length:%d\r\n",buffer_length);
	res_sd=f_write(file,(char*)buffer,buffer_length,&br);
	f_printf(file,"\r\n");
	printf("д���ļ����ش��룺%d\r\n",res_sd);	
	res_sd=f_close(file);
	printf("�ر��ļ����ش��룺%d\r\n",res_sd);
}

void connecting_server(void)
 {
	 printf("GPRSģ����ע������\r\n");
	 //Wait_CREG();
	 printf("ע��ɹ�\r\n");
	 UART2_SENDString("AT+CIPCLOSE=1");	//�ر�����
   delay_ms(100);
	 sim808_send_cmd("AT+CIPSHUT","SHUT OK",200);		//�ر��ƶ�����
	 sim808_send_cmd("AT+CGCLASS=\"B\"","OK",200);//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ��� 
	 sim808_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",200);//����PDP������,��������Э��,��������Ϣ
	 sim808_send_cmd("AT+CGATT=1","OK",200);//����GPRSҵ��
	 sim808_send_cmd("AT+CIPCSGP=1,\"CMNET\"","OK",200);//����ΪGPRS����ģʽ
	 sim808_send_cmd("AT+CIPHEAD=1","OK",200);//���ý���������ʾIPͷ(�����ж�������Դ,���ڵ�·������Ч)
	 sim808_send_cmd((u8*)IP_command,"OK",500);
	 delay_ms(100);
	 CLR_Buf2();
	 printf("���ӳɹ�\r\n");
 }
