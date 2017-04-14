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
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include <math.h>
/************************************************
************************************************/

//UCOSIIIÖĞÒÔÏÂÓÅÏÈ¼¶ÓÃ»§³ÌĞò²»ÄÜÊ¹ÓÃ£¬ALIENTEK
//½«ÕâĞ©ÓÅÏÈ¼¶·ÖÅä¸øÁËUCOSIIIµÄ5¸öÏµÍ³ÄÚ²¿ÈÎÎñ
//ÓÅÏÈ¼¶0£ºÖĞ¶Ï·şÎñ·şÎñ¹ÜÀíÈÎÎñ OS_IntQTask()
//ÓÅÏÈ¼¶1£ºÊ±ÖÓ½ÚÅÄÈÎÎñ OS_TickTask()
//ÓÅÏÈ¼¶2£º¶¨Ê±ÈÎÎñ OS_TmrTask()
//ÓÅÏÈ¼¶OS_CFG_PRIO_MAX-2£ºÍ³¼ÆÈÎÎñ OS_StatTask()
//ÓÅÏÈ¼¶OS_CFG_PRIO_MAX-1£º¿ÕÏĞÈÎÎñ OS_IdleTask()
//ÈÎÎñÓÅÏÈ¼¶
#define START_TASK_PRIO		8
//ÈÎÎñ¶ÑÕ»´óĞ¡	
#define START_STK_SIZE 		64
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB StartTaskTCB;
//ÈÎÎñ¶ÑÕ»	
__align(8) static CPU_STK START_TASK_STK[START_STK_SIZE];
//ÈÎÎñº¯Êı
void start_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define TASK1_TASK_PRIO		7
//ÈÎÎñ¶ÑÕ»´óĞ¡	
#define TASK1_STK_SIZE 		64
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB Task1_TaskTCB;
//ÈÎÎñ¶ÑÕ»	
__align(8) static CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
void task1_task(void *p_arg);


//ÈÎÎñ¶şÎª¼àÊÓÈÎÎñ
//ÈÎÎñÓÅÏÈ¼¶
#define TASK2_TASK_PRIO		8
//ÈÎÎñ¶ÑÕ»´óĞ¡	
#define TASK2_STK_SIZE 		128
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB Task2_TaskTCB;
//ÈÎÎñ¶ÑÕ»	
__align(8) static CPU_STK TASK2_TASK_STK[TASK2_STK_SIZE];
void task2_task(void *p_arg);

//////ÏûÏ¢¶ÓÁĞ////////////////////
#define LINE_MSG_NUM 2
OS_Q LINE_MSG;	//¶¨ÒåÏûÏ¢¶ÓÁĞ

//MPU6050Ğ£×¼Êı¾İ
#define ACCX0 1.6341
#define ACCY0 0.2329
#define ACCZ0 8.9055
#define GYROX0 -2.7779
#define GYROY0 -0.1290
#define GYROZ0 -0.9798

//u8 share_resource[1];   //¹²Ïí×ÊÔ´Çø

//OS_SEM	MY_SEM;		//¶¨ÒåÒ»¸öĞÅºÅÁ¿£¬ÓÃÓÚ·ÃÎÊ¹²Ïí×ÊÔ´

FATFS fatfs;													/* FatFsÎÄ¼şÏµÍ³¶ÔÏó */
FIL fnew;													/* ÎÄ¼ş¶ÔÏó */
FRESULT res_sd;                								/* ÎÄ¼ş²Ù×÷½á¹û */
uint16_t i;

const char *IP_command = "AT+CIPSTART=\"TCP\",\"123.206.41.146\",\"8083\"";
char *p1,*p2; 
char *time,*lati,*longi,*gps;

u8 file_name[13]={0};	//×Ö·û´®Êı×éÒª³õÊ¼»¯£¡£¡
char *char_filename;	//Ã¿´ÎSD¿¨ÖĞ´ò¿ªµÄÎÄ¼şÃû³Æ
char flag[2];					//sprintfÖĞµÄĞ´Èë
u32 fail_filesize = 0;		//Î´·¢ËÍ³É¹¦Êı¾İµÄÎ»ÖÃ
 
u8 Times=0,First_Int = 0,shijian=0;
u16 Heartbeat=0;
vu8 Heart_beat;//·¢ËÍĞÄÌøÖ¡±êÖ¾Î»

vu8 Timer0_start;	//¶¨Ê±Æ÷0ÑÓÊ±Æô¶¯¼ÆÊıÆ÷
u8 buffer[200];
char mpu[60];
char mpuInit[60];
float pitch,roll,yaw; 		//Å·À­½Ç
short aacx,aacy,aacz;		//¼ÓËÙ¶È´«¸ĞÆ÷Ô­Ê¼Êı¾İ
float aacxx,aacyy;
float g;
short gyrox,gyroy,gyroz;	//ÍÓÂİÒÇÔ­Ê¼Êı¾İ


u8 gps_analysis(void);
u8 NMEA_Comma_Pos(u8 *buf,u8 cx);
u32 NMEA_Pow(u8 m,u8 n);
void data_storage(void);
void connecting_server(void);
void write_SD_FLAG(u8 SD_FLAG);

//Ö÷º¯Êı
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	u8 res=1;
	u16 temp=0;
	u32 dtsize,dfsize;	//SD¿¨Ê£ÓàÈİÁ¿Óë×ÜÈİÁ¿
	delay_init();  //Ê±ÖÓ³õÊ¼»¯
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ÖĞ¶Ï·Ö×éÅäÖÃ
	uart_init(115200);   //´®¿Ú³õÊ¼»¯
	Timer2_Init_Config();
	LED_Init();         //LED³õÊ¼»¯	
	KEY_Init();			//°´¼ü³õÊ¼»¯
	MPU_Init();
	while(mpu_dmp_init())
	{
		printf("MPU6050 ERROR\r\n");
		delay_ms(500);
	}
	
	//SPI_Flash_Init();		//W25Qxx³õÊ¼»¯												  
  mem_init();				//ÄÚ´æ³Ø³õÊ¼»¯
	exfuns_init();
	f_mount(fs[0],"0:",1);	//¹ÒÔØSD¿¨
	while(SD_Initialize())	
	{
		printf("SD error\r\n");
	}
	do
	{
		temp++;
 		res=exf_getfree("0:",&dtsize,&dfsize);//µÃµ½SD¿¨Ê£ÓàÈİÁ¿ºÍ×ÜÈİÁ¿
		delay_ms(200);		   
	}while(res&&temp<5);//Á¬Ğø¼ì²â5´Î
	if(res==0)
	{	
		printf("SD¿¨OK Ê£Óà%dMB,×Ü¹²%dMB\r\n",dtsize>>10,dfsize>>10);
	}
	else
  {
		printf("SD¿¨error");
		
	}
	USART2_Init(115200);
	while(sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000))//´ò¿ªGPSµçÔ´!=0
	{
		printf("Ê§°Ü%s\r\n",USART2_RX_BUF);
		delay_ms(500);
	
	}
	//else{printf("³É¹¦%s\r\n",USART2_RX_BUF);}
	
			
	
	OSInit(&err);		    	//³õÊ¼»¯UCOSIII
	OS_CRITICAL_ENTER();	//½øÈëÁÙ½çÇø			 
	//´´½¨¿ªÊ¼ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//ÈÎÎñ¿ØÖÆ¿é
				 (CPU_CHAR	* )"start task", 		//ÈÎÎñÃû×Ö
                 (OS_TASK_PTR )start_task, 			//ÈÎÎñº¯Êı
                 (void		* )0,					//´«µİ¸øÈÎÎñº¯ÊıµÄ²ÎÊı
                 (OS_PRIO	  )START_TASK_PRIO,     //ÈÎÎñÓÅÏÈ¼¶
                 (CPU_STK   * )&START_TASK_STK[0],	//ÈÎÎñ¶ÑÕ»»ùµØÖ·
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//ÈÎÎñ¶ÑÕ»Éî¶ÈÏŞÎ»
                 (CPU_STK_SIZE)START_STK_SIZE,		//ÈÎÎñ¶ÑÕ»´óĞ¡
                 (OS_MSG_QTY  )0,					//ÈÎÎñÄÚ²¿ÏûÏ¢¶ÓÁĞÄÜ¹»½ÓÊÕµÄ×î´óÏûÏ¢ÊıÄ¿,Îª0Ê±½ûÖ¹½ÓÊÕÏûÏ¢
                 (OS_TICK	  )0,					//µ±Ê¹ÄÜÊ±¼äÆ¬ÂÖ×ªÊ±µÄÊ±¼äÆ¬³¤¶È£¬Îª0Ê±ÎªÄ¬ÈÏ³¤¶È£¬
                 (void   	* )0,					//ÓÃ»§²¹³äµÄ´æ´¢Çø
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //ÈÎÎñÑ¡Ïî
                 (OS_ERR 	* )&err);				//´æ·Å¸Ãº¯Êı´íÎóÊ±µÄ·µ»ØÖµ
	OS_CRITICAL_EXIT();	//ÍË³öÁÙ½çÇø	 
	OSStart(&err);      //¿ªÆôUCOSIII
}


//¿ªÊ¼ÈÎÎñº¯Êı
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//Í³¼ÆÈÎÎñ                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//Èç¹ûÊ¹ÄÜÁË²âÁ¿ÖĞ¶Ï¹Ø±ÕÊ±¼ä
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //µ±Ê¹ÓÃÊ±¼äÆ¬ÂÖ×ªµÄÊ±ºò
	 //Ê¹ÄÜÊ±¼äÆ¬ÂÖ×ªµ÷¶È¹¦ÄÜ,Ê±¼äÆ¬³¤¶ÈÎª1¸öÏµÍ³Ê±ÖÓ½ÚÅÄ£¬¼È1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
		
	OS_CRITICAL_ENTER();	//½øÈëÁÙ½çÇø
	//´´½¨Ò»¸öĞÅºÅÁ¿
//	OSSemCreate ((OS_SEM*	)&MY_SEM,
//                 (CPU_CHAR*	)"MY_SEM",
//                 (OS_SEM_CTR)1,		
//                 (OS_ERR*	)&err);
								 
	//´´½¨Ò»¸öÏûÏ¢¶ÓÁĞ¡¤
	OSQCreate (	(OS_Q*			) &LINE_MSG,
							(CPU_CHAR*	) "LINE MSG",
							(OS_MSG_QTY	)	LINE_MSG_NUM,
							(OS_ERR*		)	&err
	);
	
	//´´½¨TASK1ÈÎÎñ
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
	//´´½¨TASK2ÈÎÎñ
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
	OS_CRITICAL_EXIT();	//ÍË³öÁÙ½çÇø
	OSTaskDel((OS_TCB*)0,&err);	//É¾³ıstart_taskÈÎÎñ×ÔÉí
}

//ÈÎÎñ1µÄÈÎÎñº¯Êı ÏòSD¿¨ÖĞĞ´ÈëÊı¾İ£¬Í¬Ê±Í¨¹ıGPRS·¢ËÍ
void task1_task(void *p_arg)
{
	OS_ERR err;
	u8 task1_str[]="!";
	int temp;
	char *buffa;                // Device header
	char *mpua;
	char *mpuaInit;	//Î´Ğ£ÕıµÄÊı¾İ
	u8 SD_flag;
	
	u8 gps_able;
	u8 res_gprs = 1;
//	OSSemPend(&MY_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); 	//ÇëÇóĞÅºÅÁ¿
//	memcpy(share_resource,task1_str,sizeof(task1_str)); //Ïò¹²Ïí×ÊÔ´Çø¿½±´Êı¾İ
	connecting_server();
	res_gprs=sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000);	//´ò¿ªGPSµçÔ´
	printf("gps power:%d",res_gprs);
	while(1)
	{
		
		if(sim808_send_cmd("AT+CGNSINF\r\n","OK",1000)==0)
		{
			printf("ÊÕµ½GPS\r\n");
		}

//		Send_OK();
		delay_ms(500);
//»ñµÃ½âÎöÖ®ºóµÄÊı¾İ
		gps_able=gps_analysis();
		if(gps_able)	//Èô»ñÈ¡gpsÊı¾İÕıÈ·
		{							//1£ºÊı¾İ´æ´¢2£ºÊı¾İ·¢ËÍ²¢±ê¼Ç
			data_storage();//gpsÊı¾İ´æ´¢½øSD¿¨ÖĞ,ĞŞ¸ÄºóÃ¿´ÎÃ¿ÌõÊı¾İ²»Ìí¼Ó»»ĞĞ·û
			USART2_Init(115200);
			buffa=(char*)buffer;
			mpua = (char*)mpu;
			mpuaInit = (char*)mpuInit;
			mpua = strcat((char*)mpua,"\r\n\32\0");
			buffa = strcat((char*)buffa,"\r\n\32\0");
			res_gprs=sim808_send_cmd("AT+CIPSEND",">",200);
		  printf("cipsend:%d\r\n",res_gprs);	//0³É¹¦£»1Ê§°Ü
			res_gprs=sim808_send_cmd((u8*)buffa,"SEND OK",500);
			printf("send_ok:%d\r\n",res_gprs);
			//ÏòSD¿¨ÖĞ·¢ËÍ±êÖ¾Î»£¬Í¬ÊÂ·¢ËÍÏûÏ¢¶ÓÁĞ£
			if(0 == res_gprs)	//·¢ËÍ³É¹¦
			{
				SD_flag = 1;
				write_SD_FLAG(SD_flag);
			}
			if(1 == res_gprs){							//·¢ËÍÊ§°Ü
				SD_flag = 0;
				write_SD_FLAG(SD_flag);
				printf(".............·¢ËÍÏûÏ¢¶ÓÁĞ............%d\r\n",fail_filesize);
					OSQPost((OS_Q*		)&LINE_MSG,		
					(void*		)&fail_filesize,
					(OS_MSG_SIZE)4,			//·¢ËÍ×Ö½ÚÊı
					(OS_OPT		)OS_OPT_POST_FIFO+OS_OPT_POST_ALL, 	
					(OS_ERR*	)&err);
					printf(".........err=%d............\r\n",err);
					Send_OK();
					Send_OK();
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err);   //ÑÓÊ±1s
			}
			USART2_Init(115200);
			res_gprs=sim808_send_cmd("AT+CIPSEND",">",200);
			printf("cipsend:%d\r\n",res_gprs);	//0³É¹¦£»1Ê§°Ü
			res_gprs=sim808_send_cmd((u8*)mpua,"SEND OK",500);
			printf("send_ok:%d\r\n",res_gprs);
			Send_OK();
			delay_ms(500);
			memset(buffer,0,sizeof(buffer));	//Çå¿Õbuffer
			if(Heart_beat)
			{
				Send_OK();
				sim808_send_cmd("AT+CGNSPWR=1\r\n","AT",2000);
				Heart_beat=0;
			}
			
		}
		/*µÃµ½½Ç¶ÈÖµ£¬µ¥Î»¶È*/
		temp=mpu_dmp_get_data(&pitch,&roll,&yaw);
		printf("mpu_dmp_get_data=%d\r\n",temp);
		printf("pitch=%f\r\n",pitch);
		printf("roll=%f\r\n",roll);
		printf("yaw=%f\r\n",yaw);
		/*¼ÓËÙ¶È´«¸ĞÆ÷ÁéÃô¶È16384LSB/g
		ÍÓÂİÒÇÁéÃô¶È16.4LSB/(¶ÈÃ¿Ãë)*/
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//µÃµ½¼ÓËÙ¶È´«¸ĞÆ÷Êı¾İ
//		printf("aacx=%f,aacy=%f,aacz=%f ",(float)(aacx*9.8/16384)-ACCX0,(float)(aacy*9.8/16384)-ACCY0,(float)(aacz*9.8/16384));
		g = (float)(aacz*9.8/16384);
//		g = 9.8;
		aacxx = (float)(aacx*g/16384)-ACCX0-g*sin(pitch*3.1415/180)*cos(roll*3.1415/180);
		aacyy = (float)(aacy*g/16384)-ACCY0+g*sin(roll*3.1415/180)*cos(pitch*3.1415/180);
		printf("aacxx = %f, aacyy = %f\r\n", aacxx, aacyy);
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//µÃµ½ÍÓÂİÒÇÊı¾İ
		USART2_Init(115200);
		
		
		printf("......................................................................................\r\n\r\n");
	
		
	}	
}

//ÈÎÎñ2µÄÈÎÎñº¯Êı
void task2_task(void *p_arg)
{	
	OS_ERR err;
	OS_MSG_SIZE size;
	u32 *key;
	while(1)
	{
			key=OSQPend((OS_Q*			)&LINE_MSG,   
									(OS_TICK		)0,
									(OS_OPT			)OS_OPT_PEND_BLOCKING,
                  (OS_MSG_SIZE*	)&size,		
                  (CPU_TS*		)0,
                  (OS_ERR*		)&err);
		printf("......err=%d...",err);
		printf(".........ÊÕµ½£º%d....\r\n",(*key));
		delay_ms(5000);
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err);   //ÑÓÊ±1s
	}
}

u8 gps_analysis(void)	//·µ»Ø1Ö¤Ã÷³É¹¦
 {
	 u8 pos_begin;
	 u8 pos_end;
	 int i;
	 int time;
	 int date;
	 int month;
	 int year;
	 int leapyearFlag = 0;
	 pos_begin = NMEA_Comma_Pos(USART2_RX_BUF,2);
   pos_end = NMEA_Comma_Pos(USART2_RX_BUF,5);
	 if(USART2_RX_BUF[pos_begin] == '1' || USART2_RX_BUF[pos_begin] == '2')	//Èç¹û¶ÁÈ¡µÄÄê·İµÄµÚÒ»Î»Êı×ÖÊÇ2£¬Ö¤Ã÷Ê±¼äÕıÈ·
	{
	 for(i = pos_begin;i<pos_end;i++)
	 {
		 buffer[i-pos_begin] = USART2_RX_BUF[i];
	 }
	 /*
	 **UTC×ª±±¾©Ê±¼ä
	 */
	 time = (buffer[9]-'0') + 10*(buffer[8]-'0');
	 date = (buffer[7]-'0') + 10*(buffer[6]-'0');
	 month= (buffer[5]-'0') + 10*(buffer[4]-'0');
	 year = (buffer[3]-'0') + 10*(buffer[2]-'0') + 100*(buffer[1]-'0') + 1000*(buffer[0]-'0');
	 if( (year%4 == 0 && year%100 != 0)|| year%400 == 0){	
			leapyearFlag = 1;	//ÅĞ¶ÏÊÇ·ñÈòÄê
	 }
	 if(( time + 8 ) >= 24){	//ÈÕÆÚĞè¼ÓÒ»
		 if(leapyearFlag){
			switch( month ){
				case 2: if(date == 29){	
										date = 1; 
										month++;}
								else{date++;}
					break;
				case 4: case 6: case 9: case 11:
								if(date == 30){	
										date = 1; 
										month++;}
								else{date++;}
					break;
				case 12: if(date == 31){	
										date = 1; 
										month++;
										year++;}
								else{date++;}
					break;
				default: if(date == 31){	
										date = 1; 
										month++;}
								else{date++;}
					break;
			}
		 }
		 else{
			switch( month ){
				case 2: if(date == 28){	
										date = 1; 
										month++;}
								else{date++;}
					break;
				case 4: case 6: case 9: case 11:
								if(date == 30){	
										date = 1; 
										month++;}
								else{date++;}
					break;
				case 12: if(date == 31){	
										date = 1; 
										month++;
										year++;}
								else{date++;}
					break;
				default: if(date == 31){	
										date = 1; 
										month++;}
								else{date++;}
					break;
			}
		 
		 } 
	 }
	 else{time = time + 8;}
	 buffer[3] = year%10 + '0'; year=year/10;
	 buffer[2] = year%10 + '0'; year=year/10;
	 buffer[1] = year%10 + '0'; year=year/10;
	 buffer[0] = year%10 + '0'; year=year/10;
	 
	 buffer[5] = month%10 + '0'; buffer[4]=month/10 + '0';
	 
	 buffer[7] = date%10 + '0'; buffer[6]=date/10 + '0';
	 buffer[9] = time%10 + '0'; buffer[8]=time/10 + '0';
//	 printf("USART2_RX_BUF:%s\r\n",USART2_RX_BUF);
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
		if(*buf=='*')return 0XFF;//Óöµ½'*'»òÕß·Ç·¨×Ö·û,Ôò²»´æÔÚµÚcx¸ö¶ººÅ
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

//m^nº¯Êı
//·µ»ØÖµ:m^n´Î·½.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

////½«Êı¾İ°´Ìõ´æ´¢µ½SD¿¨ÖĞ£¬Ã¿´Î²»Ìí¼Ó\r\n,Í¬Ê±»ñÈ¡¸ÃÌõÊı¾İÔÚÎÄ¼şÖĞµÄÎ»ÖÃ£¬¼´µÚ¼¸Ìõ
void data_storage(void)
{
	u8 temp;
	u8 buffer_length;
	
	
	
	SD_Initialize();
//	printf("buffer:%s\r\n",buffer);
	for(temp=0;temp<8;temp++)
	{
		file_name[temp]=buffer[temp];
	}
	file_name[8]='.';file_name[9]='t';file_name[10]='x';file_name[11]='t';
	
	printf("%s\r\n",(char*)file_name);
	//½«»ñÈ¡µ½µÄÄêÔÂÈÕĞÅÏ¢×÷ÎªÎÄ¼şÃû£¬´´½¨txtÎÄ¼ş
	char_filename=(char*)file_name;
	//printf("char_filename:%s\r\n",char_filename);
	//´ò¿ª¸ÃÎÄ¼ş£¬½«bufferĞ´ÈëÎÄ¼şÖĞ
	res_sd=f_open(file, char_filename, FA_OPEN_ALWAYS|FA_WRITE);
	printf("open:%d\r\n",res_sd);
	printf("file_size: %d\r\n",(int)(*file).fsize);
	f_lseek(file,(*file).fsize);
	buffer_length = strlen((char*)buffer);
	printf("buffer_length:%d\r\n",buffer_length);
	res_sd=f_write(file,(char*)buffer,buffer_length,&br);
	sprintf(mpu,"%f,%f,%f,%f,%f,%f",aacxx,aacyy,(float)(aacz*9.8/16384)-ACCZ0,(float)(gyrox/16.4)-GYROX0,(float)(gyroy/16.4)-GYROY0,(float)(gyroz/16.4)-GYROZ0);
	res_sd=f_write(file,(char*)mpu,strlen(mpu),&br);
	printf("write:%d\r\n",res_sd);	
	res_sd=f_close(file);
	printf("close:%d\r\n",res_sd);
}

void write_SD_FLAG(u8 SD_FLAG)
{
	u8 num;
	num=SD_FLAG;
	SD_Initialize();
	res_sd=f_open(file, char_filename, FA_OPEN_ALWAYS|FA_WRITE);
	printf("FLAG´ò¿ªÎÄ¼ş·µ»Ø´úÂë£º%d\r\n",res_sd);
	printf("FLAG_file_size: %ld\r\n",(*file).fsize);
	f_lseek(file,(*file).fsize);
	sprintf(flag,",%d",num);
	res_sd=f_write(file,(char*)flag,strlen(flag),&br);
	f_printf(file,"\r\n");
	if(0 == num)
		fail_filesize = (*file).fsize;
	printf("FLAGĞ´ÈëÎÄ¼ş·µ»Ø´úÂë£º%d\r\n",res_sd);	
	res_sd=f_close(file);
	printf("FLAG¹Ø±ÕÎÄ¼ş·µ»Ø´úÂë£º%d\r\n",res_sd);
}

void connecting_server(void)
 {
	 printf("GPRSÄ£¿éÔÚ×¢²áÍøÂç\r\n");
	 //Wait_CREG();
	 printf("×¢²á³É¹¦\r\n");
	 UART2_SENDString("AT+CIPCLOSE=1");	//¹Ø±ÕÁ¬½Ó
   delay_ms(100);
	 sim808_send_cmd("AT+CIPSHUT","SHUT OK",200);		//¹Ø±ÕÒÆ¶¯³¡¾°
	 sim808_send_cmd("AT+CGCLASS=\"B\"","OK",200);//ÉèÖÃGPRSÒÆ¶¯Ì¨Àà±ğÎªB,Ö§³Ö°ü½»»»ºÍÊı¾İ½»»» 
	 sim808_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",200);//ÉèÖÃPDPÉÏÏÂÎÄ,»¥ÁªÍø½ÓĞ­Òé,½ÓÈëµãµÈĞÅÏ¢
	 sim808_send_cmd("AT+CGATT=1","OK",200);//¸½×ÅGPRSÒµÎñ
	 sim808_send_cmd("AT+CIPCSGP=1,\"CMNET\"","OK",200);//ÉèÖÃÎªGPRSÁ¬½ÓÄ£Ê½
	 sim808_send_cmd("AT+CIPHEAD=1","OK",200);//ÉèÖÃ½ÓÊÕÊı¾İÏÔÊ¾IPÍ·(·½±ãÅĞ¶ÏÊı¾İÀ´Ô´,½öÔÚµ¥Â·Á¬½ÓÓĞĞ§)
	 sim808_send_cmd((u8*)IP_command,"OK",500);
	 delay_ms(100);
	 CLR_Buf2();
	 printf("Á¬½Ó³É¹¦\r\n");
 }
