/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "ch32v30x.h"
#include <rtthread.h>
#include <rthw.h>
#include "drivers/pin.h"
#include "drv_gpio.h"
#include <board.h>
#include "debug.h"
#include <string.h>
#include "drv_usart.h"
#include "at.h"

#include "esp8266/esp8266.h"
#include "applications/ov/ov.h"
#include "applications/sm4/sm4.h"


#define IMAGE_SIZE 8192

sm4_ctx ctx;
uint8_t gkey[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10};


ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t serial_thread_stack[ 512 ];//串口监听线程栈

//ALIGN(RT_ALIGN_SIZE)
//static rt_uint8_t send_thread_stack[ 512 ];//发送数据线程栈

static rt_thread_t serial_thread;//tf串口线程,动态线程
//static rt_thread_t send_thread;//esp发送加密数据线程
volatile rt_thread_t reset_thread;//esp重新配置线程

static rt_device_t tf_serial;//tf设备

static struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;//tf串口初始化配置参数

//发送图像状态标志
volatile u_int8_t key_flag;
extern u_int8_t esp_send_flag;

char WIFI_ID [25];
char WIFI_Password[25];
char UDP_Server_IP[25];
char UDP_Server_Port[25];

//tf串口处理线程使用变量
uint8_t Uart_Last_Recieve;
uint8_t Uart_Now_Recieve;
uint8_t WIFI_ID_Rec =0;
uint8_t WIFI_ID_Rec_Data =0;
uint8_t WIFI_Pass_Rec =0;
uint8_t WIFI_Pass_Rec_Data =0;
uint8_t UDP_Server_IP_Rec =0;
uint8_t UDP_Server_IP_Rec_Data =0;
uint8_t UDP_Server_Port_Rec =0;
uint8_t UDP_Server_Port_Rec_Data =0;
volatile uint8_t Rec_Over = 0;
uint8_t Cnt =0;
/*
 *串口中断回调函数
 */
static struct rt_semaphore rx_sem;//串口用于接收消息的信号量
static rt_err_t tf_uart_input(rt_device_t dev, rt_size_t size){
    // 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&rx_sem);
    return RT_EOK;
}
/*
 * * 串口初始化函数
 * */
static int tf_uart_init(){
    // 查找系统中的串口设备
    tf_serial = rt_device_find(TF_UART_NAME);
    if (!tf_serial){
        rt_kprintf("find %s failed!\n", TF_UART_NAME);
        return RT_ERROR;
    }
    // 初始化信号量
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    //配置串口参数
    config.baud_rate = BAUD_RATE_9600;        //波特率为 9600
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    rt_device_control(tf_serial, RT_DEVICE_CTRL_CONFIG, &config);
    // 以中断接收及轮询发送模式打开串口设备
    rt_device_open(tf_serial, RT_DEVICE_FLAG_INT_RX);
    // 设置接收回调函数
    rt_device_set_rx_indicate(tf_serial, tf_uart_input);
    return RT_EOK;
}

static void serial_thread_entry(void *parameter){
    char ch;
    while (1){
        // 从串口读取一个字节的数据，没有读取到则等待接收信号量
        while (rt_device_read(tf_serial, -1, &ch, 1) != 1){
            //阻塞等待接收信号量，等到信号量后再次读取数据
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        rt_device_write(tf_serial, 0, &ch, 1);
        Uart_Now_Recieve = ch;
        if( (Uart_Now_Recieve == 0x28) && (Uart_Last_Recieve == 0x27)){
            rt_kprintf("send\n");
            key_flag =  1;   //将按键标志位置1
            DVP_enable();
        }
        else if( (Uart_Now_Recieve == 0x05) && (Uart_Last_Recieve == 0xFF)){
            WIFI_ID_Rec = 1;
        }
        else if( (Uart_Now_Recieve == 0x06) && (Uart_Last_Recieve == 0xFF)){//清除WIFI_ID多存入的0XFF
            WIFI_ID_Rec = 0;
            WIFI_ID_Rec_Data = 0;
            WIFI_Pass_Rec = 1;
            WIFI_ID[Cnt-1] = '\0';
            Cnt = 0;
        }
        else if( (Uart_Now_Recieve == 0x07) && (Uart_Last_Recieve == 0xFF)){
            WIFI_Pass_Rec = 0;
            WIFI_Pass_Rec_Data= 0;
            UDP_Server_IP_Rec = 1;
            WIFI_Password[Cnt-1] = '\0';
            Cnt = 0;
        }
        else if( (Uart_Now_Recieve == 0x08) && (Uart_Last_Recieve == 0xFF)){
            UDP_Server_IP_Rec =0;
            UDP_Server_IP_Rec_Data =0;
            UDP_Server_Port_Rec = 1;
            UDP_Server_IP[Cnt-1] = '\0';
            Cnt = 0;
        }
        else if( (Uart_Now_Recieve == 0x09) && (Uart_Last_Recieve == 0xFF)){ //尾帧
            UDP_Server_Port_Rec_Data =0;
            UDP_Server_Port_Rec = 0;
            UDP_Server_Port[Cnt-1] = '\0';
            Cnt = 0;
            Rec_Over = 1;
        }
        else { ; }
        if(WIFI_ID_Rec){
            if(Uart_Last_Recieve ==  0x05){
                WIFI_ID_Rec_Data = 1;
            }
            if(WIFI_ID_Rec_Data){
                WIFI_ID[Cnt] = Uart_Now_Recieve;
                Cnt ++;
            }
        }
        else if(WIFI_Pass_Rec){
            if(Uart_Last_Recieve ==  0x06){
                WIFI_Pass_Rec_Data = 1;
            }
            if(WIFI_Pass_Rec_Data){
                WIFI_Password[Cnt] = Uart_Now_Recieve;
                Cnt ++;
            }
        }
        else if(UDP_Server_IP_Rec){
            if(Uart_Last_Recieve ==  0x07){
                UDP_Server_IP_Rec_Data = 1;
            }
            if(UDP_Server_IP_Rec_Data){
                UDP_Server_IP[Cnt] = Uart_Now_Recieve;
                Cnt ++;
            }
        }
        else if(UDP_Server_Port_Rec){
            if(Uart_Last_Recieve ==  0x08){
                UDP_Server_Port_Rec_Data = 1;
            }
            if(UDP_Server_Port_Rec_Data){
                UDP_Server_Port[Cnt] = Uart_Now_Recieve;
                Cnt ++;
            }
        }
        if(Rec_Over == 1){
            //Reset_Esp8266();
            rt_thread_mdelay(100);
            reset_thread = rt_thread_create("reset_thread",esp8266_init_Reconfiguring,RT_NULL,512,4,100);
            if(reset_thread != NULL){
                rt_kprintf("resetth Y\n");
                rt_thread_startup(reset_thread);
                rt_kprintf("resetth S\n");
            }
            else{
                rt_kprintf("resetth F");
            }
        }
        Uart_Last_Recieve = Uart_Now_Recieve;
    }
}

uint32_t  JPEG_DVPDMAaddr0 = 0x20005000;
uint32_t  JPEG_DVPDMAaddr1 = 0x20005000 + OV2640_JPEG_WIDTH*2;//每一字节数据实际占两字节RAM  一个像素 2byte = 16bit
uint32_t  RGB565_DVPDMAaddr0 = 0x20005000;
uint32_t  RGB565_DVPDMAaddr1 = 0x20005000 + RGB565_COL_NUM;
volatile uint32_t frame_cnt = 0;
volatile uint32_t addr_cnt = 0;
volatile uint32_t href_cnt = 0;
//中断里使用的变量加 volatile 成为全局变量。

/*-------------------------------DVP-----------------------*/
void DVP_Init(void){
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DVP, ENABLE);// 打开DVP模块的时钟信号
    DVP->CR0 &= ~RB_DVP_MSK_DAT_MOD;// 清除DVP配置寄存器CR0
#if (DVP_Work_Mode == JPEG_MODE)
    DVP->CR0 |= RB_DVP_D10_MOD | RB_DVP_V_POLAR | RB_DVP_JPEG;// 设置DVP的工作模式（10bit位宽)，同步信号极性（低有效的VSYNC信号），打开DVP的JPEG模式
    DVP->CR1 &= ~(RB_DVP_ALL_CLR| RB_DVP_RCV_CLR);// 配置寄存器CR0清除DVP缓存和标志位
    DVP->COL_NUM = OV2640_JPEG_WIDTH;// 设置图片信号宽度
    // 配置DMA的目标地址
    DVP->DMA_BUF0 = JPEG_DVPDMAaddr0;        //DMA addr0
    DVP->DMA_BUF1 = JPEG_DVPDMAaddr1;        //DMA addr1
#endif
    // 设置DVP帧捕获率
    DVP->CR1 &= ~RB_DVP_FCRC;
    DVP->CR1 |= DVP_RATE_100P ;//DVP_RATE_25P;  //25% DVP_RATE_100P
    //Interupt Enable
    DVP->IER |= RB_DVP_IE_STP_FRM;
    DVP->IER |= RB_DVP_IE_FIFO_OV;
    DVP->IER |= RB_DVP_IE_FRM_DONE;
    DVP->IER |= RB_DVP_IE_ROW_DONE;
    DVP->IER |= RB_DVP_IE_STR_FRM;

    NVIC_InitStructure.NVIC_IRQChannel = DVP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    DVP->CR1 &= ~RB_DVP_DMA_EN;  //enable DMA
    DVP->CR0 &= ~RB_DVP_ENABLE;  //enable DVP
}
u32 DVP_ROW_cnt=0;
//DVP中断
void DVP_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast"))); //设置中断
void DVP_IRQHandler(void){
    uint8_t data[16]; //源数据
    uint8_t data_en[16];//加密后的数据
    uint8_t count;
    uint8_t send_state;
    uint8_t start_flag;
    uint8_t end_flag;
    uint8_t last_data,new_data;
    send_state = 0x0;
    new_data = 0x0;
    last_data = 0x0;
    count = 0x0;
    start_flag = 0;
    end_flag = 0;
    if (DVP->IFR & RB_DVP_IF_ROW_DONE){
        /* Write 0 clear 0 */
        DVP->IFR &= ~RB_DVP_IF_ROW_DONE;  //clear Interrupt
#if (DVP_Work_Mode == JPEG_MODE)
        href_cnt++;
        if (addr_cnt%2){     //buf1 done
            addr_cnt++;
            DVP->DMA_BUF1 += OV2640_JPEG_WIDTH *4;
        }
        else{                //buf0 done
            addr_cnt++;
            DVP->DMA_BUF0 += OV2640_JPEG_WIDTH *4;
        }
#endif
    }
    if (DVP->IFR & RB_DVP_IF_FRM_DONE){
        DVP->IFR &= ~RB_DVP_IF_FRM_DONE;  //clear Interrupt
#if (DVP_Work_Mode == JPEG_MODE)
        DVP->CR0 &= ~RB_DVP_ENABLE;       //disable DVP
        //Use uart2 send JPEG data.
        UINT32  i;
        UINT16  val;
        //UINT8   j;
        href_cnt = href_cnt*OV2640_JPEG_WIDTH;
        if(key_flag == 1){
            for(i = 0; i < href_cnt; i++){
                val = *(UINT16*)(0x20005000+i*2);
                last_data = new_data;//上个数据缓存
                new_data = (UINT8)(val>>2);//新数据缓存
                if(new_data == 0xD8 && last_data == 0xFF){//帧头检测
                    start_flag = 1;
                    send_state = 1;
                    count = 0;
                }
                else if(new_data == 0xD9 && last_data == 0xFF){//帧尾检测
                    end_flag = 1;
                }
                else ;
                if(send_state == 1){ //帧头发送开始
                    if(start_flag == 1){
                        at_client_send(&last_data,1);
                        at_client_send(&new_data,1);
                        start_flag = 0;}
                    else if(end_flag == 1){ //帧尾结束发送
                        data[count-1] = 0x00;
                        data[count] = 0x00;
                        count++;
                        if(count == 16){
                            send_state = 0;
                            key_flag = 0;
                        }
                        else {
                            new_data = 0x0;
                        }
                    }
                    else{   //图像中间数据发送
                        data[count] = new_data;
                        count++;
                    }
                }
                if(count == 16){
                    sm4_encrypt(data,data_en,&ctx);
                   // image_data[counter++] = data_en;
                    at_client_send(data_en,16);
                    if(end_flag == 1){
                        last_data = 0xFF;
                        new_data = 0xD9;
                        //image_data[counter++] = last_data;
                        //image_data[counter++] = new_data;
                        at_client_send(&last_data,1);
                        at_client_send(&new_data,1);
                        end_flag = 0;
                        DVP_disable();
                    }
                    count = 0;
                }
            }
        }
        else{
            send_state = 0;
        }
        DVP->CR0 |= RB_DVP_ENABLE;  //enable DVP
        DVP->DMA_BUF0 = JPEG_DVPDMAaddr0;        //DMA addr0
        DVP->DMA_BUF1 = JPEG_DVPDMAaddr1;        //DMA addr1
        href_cnt = 0;
        addr_cnt =0;
#endif
    }
    if (DVP->IFR & RB_DVP_IF_STR_FRM){
        DVP->IFR &= ~RB_DVP_IF_STR_FRM;  //clear Interrupt
        frame_cnt++;
    }
    if (DVP->IFR & RB_DVP_IF_STP_FRM){
        DVP->IFR &= ~RB_DVP_IF_STP_FRM;  //clear Interrupt
    }
    if (DVP->IFR & RB_DVP_IF_FIFO_OV){
        DVP->IFR &= ~RB_DVP_IF_FIFO_OV;   //clear Interrupt
        rt_kprintf("FIFO OV\r\n");
    }
}
/* Global define */
//TF屏串口定义

int main(void){
    rt_err_t result;

    rt_kprintf("MCU: CH32V307\n");
    rt_kprintf("SysClk: %dHz\n",SystemCoreClock);

    sm4_set_key(gkey, &ctx);
    tf_uart_init();
    DVP_Init();

    key_flag = 0x0;
    // 初始化串口线程,静态线程
    result = rt_thread_init(serial_thread,
                            "serial_thread",
                            serial_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&serial_thread_stack[0],
                            sizeof(serial_thread_stack),
                            5,
                            10);
    // 启动串口线程
    if (result == RT_EOK){
        rt_kprintf("serialth Y\n");
        rt_thread_startup(serial_thread);
        rt_kprintf("serialth S\n");
    }
    else{
        rt_kprintf("serialth F\n");
    }
    return 0;
}
