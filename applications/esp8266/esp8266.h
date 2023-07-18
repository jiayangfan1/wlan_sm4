/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-12     LQS       the first version
 */
#ifndef APPLICATIONS_ESP8266_ESP8266_H_
#define APPLICATIONS_ESP8266_ESP8266_H_


#define array_sizeof(a) (sizeof(a) / sizeof(a[0]))

extern volatile u_int8_t Rec_Over;
extern volatile rt_thread_t reset_thread;//esp重新配置线程

/* 服务函数结构体*/
typedef struct typ_Esp8266_handler
{
  char  *CmdString; //命令字符串
  void  (*CmdOperate)(char *Cmd,float Cycle);//命令执行的功能操作
} typEsp8266_t;
int Reset_Esp8266(void);
void esp8266_init_Reconfiguring(void *parameter);
//extern void esp8266_init_Reconfiguring(void);

#endif /* APPLICATIONS_ESP8266_ESP8266_H_ */
