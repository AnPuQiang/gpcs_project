#ifndef __SIM808_H__
#define __SIM808_H__	 
#include "sys.h"

u8* sim808_check_cmd(u8 *str);//��������ķ���ֵ
u8 sim808_send_cmd(u8 *cmd,u8 *ack,u16 waittime);//ָ���
u8 sim808_gsminfo__show(u16 x,u16 y);//��ʾgsmģ����Ϣ
void Wait_CREG(void);
void Send_OK(void);
#endif

