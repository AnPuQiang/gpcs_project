#ifndef __SIM808_H__
#define __SIM808_H__	 
#include "sys.h"

u8* sim808_check_cmd(u8 *str);//检验命令的返回值
u8 sim808_send_cmd(u8 *cmd,u8 *ack,u16 waittime);//指令发送
u8 sim808_gsminfo__show(u16 x,u16 y);//显示gsm模块信息
void Wait_CREG(void);
void Send_OK(void);
#endif

