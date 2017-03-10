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

//UCOSIII中以下优先级用户程序不能使用，ALIENTEK
//将这些优先级分配给了UCOSIII的5个系统内部任务
//优先级0：中断服务服务管理任务 OS_IntQTask()
//优先级1：时钟节拍任务 OS_TickTask()
//优先级2：定时任务 OS_TmrTask()
//优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
//优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()
//任务优先级
#define START_TASK_PRIO		8
//任务堆栈大小	
#define START_STK_SIZE 		64
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
__align(8) static CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//任务优先级
#define TASK1_TASK_PRIO		7
//任务堆栈大小	
#define TASK1_STK_SIZE 		64
//任务控制块
OS_TCB Task1_TaskTCB;
//任务堆栈	
__align(8) static CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
void task1_task(void *p_arg);


//任务二为监视任务
//任务优先级
#define TASK2_TASK_PRIO		9
//任务堆栈大小	
#define TASK2_STK_SIZE 		128
//任务控制块
OS_TCB Task2_TaskTCB;
//任务堆栈	
__align(8) static CPU_STK TASK2_TASK_STK[TASK2_STK_SIZE];
void task2_task(void *p_arg);

u8 share_resource[1];   //共享资源区

OS_SEM	MY_SEM;		//定义一个信号量，用于访问共享资源

FATFS fatfs;													/* FatFs文件系统对象 */
FIL fnew;													/* 文件对象 */
FRESULT res_sd;                								/* 文件操作结果 */
uint16_t i;

const char *IP_command = "AT+CIPSTART=\"TCP\",\"123.206.41.146\",\"8083\"";
char *p1,*p2; 
char *time,*lati,*longi,*gps;
 
u8 Times=0,First_Int = 0,shijian=0;
u16 Heartbeat=0;
vu8 Heart_beat;//发送心跳帧标志位

vu8 Timer0_start;	//定时器0延时启动计数器
u8 buffer[200];


u8 gps_analysis(void);
u8 NMEA_Comma_Pos(u8 *buf,u8 cx);
u32 NMEA_Pow(u8 m,u8 n);
void data_storage(void);
void connecting_server(void);

//主函数
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	u8 res=1;
	u16 temp=0;
	u32 dtsize,dfsize;	//SD卡剩余容量与总容量
	delay_init();  //时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置
	uart_init(115200);   //串口初始化

	Timer2_Init_Config();
	LED_Init();         //LED初始化	
	KEY_Init();			//按键初始化
	//SPI_Flash_Init();		//W25Qxx初始化												  
  mem_init();				//内存池初始化
	exfuns_init();
	f_mount(fs[0],"0:",1);	//挂载SD卡
	while(SD_Initialize())	
	{
		printf("SD error\r\n");
	}
	do
	{
		temp++;
 		res=exf_getfree("0:",&dtsize,&dfsize);//得到SD卡剩余容量和总容量
		delay_ms(200);		   
	}while(res&&temp<5);//连续检测5次
	if(res==0)
	{	
		printf("SD卡OK 剩余%dMB,总共%dMB\r\n",dtsize>>10,dfsize>>10);
	}
	else
  {
		printf("SD卡error");
		
	}
	USART2_Init(115200);
	if(sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000))//打开GPS电源!=0
	{printf("失败%s\r\n",USART2_RX_BUF);}
	else{printf("成功%s\r\n",USART2_RX_BUF);}
	
			
	
	OSInit(&err);		    	//初始化UCOSIII
	OS_CRITICAL_ENTER();	//进入临界区			 
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);      //开启UCOSIII
}


//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
		
	OS_CRITICAL_ENTER();	//进入临界区
	//创建一个信号量
	OSSemCreate ((OS_SEM*	)&MY_SEM,
                 (CPU_CHAR*	)"MY_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
	//创建TASK1任务
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
	//创建TASK2任务
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
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}

//任务1的任务函数 向SD卡中写入数据
void task1_task(void *p_arg)
{
	OS_ERR err;
	u8 task1_str[]="!";
	char buf[100];
	char *buffa;
	u8 gps_able;
	u8 res_gprs;
	OSSemPend(&MY_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); 	//请求信号量
	memcpy(share_resource,task1_str,sizeof(task1_str)); //向共享资源区拷贝数据
//	USART2_Init(115200);
	connecting_server();
	res_gprs=sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000);	//打开GPS电源
	printf("gps power:%d",res_gprs);
	while(1)
	{
		USART2_Init(115200);
		if(sim808_send_cmd("AT+CGNSINF\r\n","OK",1000)==0)
		{
			printf("收到GPS\r\n");
		}
//		Send_OK();
		delay_ms(500);
//获得解析之后的数据
		gps_able=gps_analysis();
		if(gps_able)	//若获取gps数据正确
		{							//1：数据存储2：数据发送并标记
			data_storage();//gps数据存储进SD卡中
			USART2_Init(115200);
			buffa=(char*)buffer;
			buffa = strcat((char*)buffa,"\r\n\32\0");
			res_gprs=sim808_send_cmd("AT+CIPSEND",">",200);
			printf("cipsend:%d\r\n",res_gprs);	//0成功；1失败
			res_gprs=sim808_send_cmd((u8*)buffa,"SEND OK",800);
			printf("send_ok:%d\r\n",res_gprs);
			delay_ms(500);
			memset(buffer,0,sizeof(buffer));	//清空buffer
			if(Heart_beat)
			{
				Send_OK();
				sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000);
				Heart_beat=0;
			}
			
		}
//		printf("%s\r\n",share_resource);	//串口输出共享资源区数据	
		OSSemPost (&MY_SEM,OS_OPT_POST_1,&err);				//发送信号量
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err);   //延时1s
	}
}

//任务2的任务函数
void task2_task(void *p_arg)
{	
	OS_ERR err;
//	u8 task2_str[]="!";
//	while(1)
//	{
//		printf("\r\n任务2:\r\n");
//		OSSemPend(&MY_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); 	//请求信号量
//		memcpy(share_resource,task2_str,sizeof(task2_str));	//向共享资源区拷贝数据
//		delay_ms(300);
//		printf("%s\r\n",share_resource);	//串口输出共享资源区数据		
//		OSSemPost (&MY_SEM,OS_OPT_POST_1,&err);				//发送信号量
//		LED1 = ~LED1;
//		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err);   //延时1s
//	}
}

u8 gps_analysis(void)	//返回1证明成功
 {
	 u8 pos_begin;
	 u8 pos_end;
	 int i;
	 pos_begin = NMEA_Comma_Pos(USART2_RX_BUF,2);
   pos_end = NMEA_Comma_Pos(USART2_RX_BUF,5);
	 if(USART2_RX_BUF[pos_begin] == '1' || USART2_RX_BUF[pos_begin] == '2')	//如果读取的年份的第一位数字是2，证明时间正确
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
		if(*buf=='*')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

//m^n函数
//返回值:m^n次方.
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
	u8 file_name[13]={0};	//字符串数组要初始化！！
	char *char_filename;
	SD_Initialize();
//	printf("buffer:%s\r\n",buffer);
	for(temp=0;temp<8;temp++)
	{
		file_name[temp]=buffer[temp];
	}
	file_name[8]='.';file_name[9]='t';file_name[10]='x';file_name[11]='t';
	
	printf("file_name:%s\r\n",(char*)file_name);
	//将获取到的年月日信息作为文件名，创建txt文件
	char_filename=(char*)file_name;
	printf("char_filename:%s\r\n",char_filename);
	//打开该文件，将buffer写入文件中
	res_sd=f_open(file, char_filename, FA_OPEN_ALWAYS|FA_WRITE);
	printf("打开文件返回代码：%d\r\n",res_sd);
	printf("file_size: %d\r\n",(int)(*file).fsize);
	f_lseek(file,(*file).fsize);
	buffer_length = strlen((char*)buffer);
	printf("buffer_length:%d\r\n",buffer_length);
	res_sd=f_write(file,(char*)buffer,buffer_length,&br);
	f_printf(file,"\r\n");
	printf("写入文件返回代码：%d\r\n",res_sd);	
	res_sd=f_close(file);
	printf("关闭文件返回代码：%d\r\n",res_sd);
}

void connecting_server(void)
 {
	 printf("GPRS模块在注册网络\r\n");
	 //Wait_CREG();
	 printf("注册成功\r\n");
	 UART2_SENDString("AT+CIPCLOSE=1");	//关闭连接
   delay_ms(100);
	 sim808_send_cmd("AT+CIPSHUT","SHUT OK",200);		//关闭移动场景
	 sim808_send_cmd("AT+CGCLASS=\"B\"","OK",200);//设置GPRS移动台类别为B,支持包交换和数据交换 
	 sim808_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",200);//设置PDP上下文,互联网接协议,接入点等信息
	 sim808_send_cmd("AT+CGATT=1","OK",200);//附着GPRS业务
	 sim808_send_cmd("AT+CIPCSGP=1,\"CMNET\"","OK",200);//设置为GPRS连接模式
	 sim808_send_cmd("AT+CIPHEAD=1","OK",200);//设置接收数据显示IP头(方便判断数据来源,仅在单路连接有效)
	 sim808_send_cmd((u8*)IP_command,"OK",500);
	 delay_ms(100);
	 CLR_Buf2();
	 printf("连接成功\r\n");
 }
