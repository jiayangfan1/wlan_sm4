/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-12     LQS       the first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <rtconfig.h>
#include <rtdbg.h>
#include <at_device_esp8266.h>
#include "esp8266.h"
#include <string.h>
#include<drv_gpio.h>

extern char WIFI_ID [25];
extern char WIFI_Password[25];
extern char UDP_Server_IP[25];
extern char UDP_Server_Port[25];

/*重启esp8266 */
int Reset_Esp8266(void){
    rt_pin_mode(GET_PIN(D, 11), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D, 11), 0);
    rt_thread_mdelay(200);
    rt_pin_write(GET_PIN(D,11), 1);
    rt_kprintf("Reset_Esp8266\r\n");
    rt_strncpy (WIFI_ID,ESP8266_SAMPLE_WIFI_SSID,20);
    rt_strncpy (WIFI_Password,ESP8266_SAMPLE_WIFI_PASSWORD,20);
    rt_strncpy (UDP_Server_IP,SERVER_IP,20);
    rt_strncpy (UDP_Server_Port,SERVER_PORT,10);

    return 0;
}
//上电初始化
INIT_PREV_EXPORT(Reset_Esp8266);

/*esp8266结构体回调函数 */
void CallBack(char *Cmd,float Cycle)
{
}

/*初始化esp8266命令结构体，回调函数暂时没用到 */
 typEsp8266_t typEsp8266CmdTable[]={
    //{"AT",                                          CallBack},          //0：测试连接
    {"AT+CWMODE=3",                                 CallBack},          //1：工作在客户端模式
    {"AT+RST",                                      CallBack},          //2：复位
    {"",                                            CallBack},          //3: 连接无线
    {"AT+CIFSR",                                    CallBack},          //4: 获取自身的IP及Mac地址
    {"AT+CIPMUX=0",                                 CallBack},          //5：关闭多连接
    {"",                                            CallBack},          //6：连接服务器IP端口
    {"AT+CIPMODE=1",                                CallBack},          //7：透传模式
    {"AT+CIPSEND",                                  CallBack}           //8：启动透传，之后esp8266进入透传状态，发送的数据将转发向服务器，不再识别AT指令
};

/*配置esp8266线程 */
void esp8266_init_Reconfiguring(void *parameter){
    rt_err_t result;
    char cCmd1 [100];
    char cCmd2 [100];
    char ch[] = "hello";
    sprintf (cCmd1, "AT+CWJAP=\"%s\",\"%s\"", WIFI_ID, WIFI_Password);
    typEsp8266CmdTable[2].CmdString = cCmd1;
    sprintf (cCmd2, "AT+CIPSTART=\"UDP\",\"%s\",%s", UDP_Server_IP, UDP_Server_Port);
    typEsp8266CmdTable[5].CmdString = cCmd2;
    rt_kprintf("%s\n",typEsp8266CmdTable[2].CmdString);
    rt_kprintf("%s\n",typEsp8266CmdTable[5].CmdString);
    at_response_t resp = RT_NULL;
    const char *line_buffer = RT_NULL;

    /* 创建响应结构体，设置最大支持响应数据长度为 512 字节，响应数据行数无限制，超时时间为8 秒 */
    resp = at_create_resp(512, 0, rt_tick_from_millisecond(8000));
    if (!resp){
        rt_kprintf("No memory for response structure!");
        //return -RT_ENOMEM;
    }
    /* 发送 AT 命令并接收 AT Server 响应数据，数据及信息存放在 resp 结构体中并打印出来 */
    for(int i=0;i<array_sizeof(typEsp8266CmdTable);i++){
        if (at_exec_cmd(resp, typEsp8266CmdTable[i].CmdString) != RT_EOK){               //发送AT指令
            rt_kprintf("%s指令没收到esp8266的回应\n",typEsp8266CmdTable[i].CmdString);
        }
        else{
            for(rt_size_t line_num = 1; line_num <= resp->line_counts; line_num++) {     //将回应的数据打印出来
                if((line_buffer = at_resp_get_line(resp, line_num)) != RT_NULL){
                    rt_kprintf("%s line%d Response buf: %s\n",typEsp8266CmdTable[i].CmdString,line_num, line_buffer);
                }
                else{
                    rt_kprintf("Parse line buffer error!");
                }
            }
        }
    }
    rt_thread_mdelay(20);
    at_delete_resp(resp);
    at_client_send(ch,5);
    Rec_Over = 0;
    result = rt_thread_delete(reset_thread);
    if (result == RT_EOK){
        rt_kprintf("resetth D\n");
    }
    else{
        rt_kprintf("resetth F\n");
    }
}

