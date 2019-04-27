/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Technology Co., LTD
**
**                                 http://www.zlgmcu.com
**
**      广州周立功单片机科技有限公司所提供的所有服务内容旨在协助客户加速产品的研发进度，在服务过程中所提供
**  的任何程序、文档、测试结果、方案、支持等资料和信息，都仅供参考，客户有权不使用或自行参考修改，本公司不
**  提供任何的完整性、可靠性等保证，若在客户使用过程中因任何原因造成的特别的、偶然的或间接的损失，本公司不
**  承担任何责任。
**                                                                        ——广州周立功单片机科技有限公司
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.c
** Last modified Date:  2012-12-18
** Last Version:        V1.00
** Descriptions:        The main() function example template
**
**--------------------------------------------------------------------------------------------------------
** Created by:          lixiang
** Created date:        2014-10-17
** Version:             V1.00
** Descriptions:        整理模板，添加用户应用程序
**
*********************************************************************************************************/
#include "LPC8xx.h"
///*********************************************************************************************************
//** Function name:       SysTick_Handler
//** Descriptions:        系统节拍定时器中断服务函数
//** input parameters:    无
//** output parameters:   无
//** Returned value:      无
//*********************************************************************************************************/
//extern void xPortSysTickHandler(void);
//void SysTick_Handler (void)
//{
//    xPortSysTickHandler();
//    //LPC_GPIO_PORT->NOT[0] |= LED;                                       /* SysTick中断，LED状态翻转     */
//}
/*********************************************************************************************************
** Function name:       SysTickInit
** Descriptions:        系统节拍定时器初始化
**                      uiUsTime：定时 uiUsTime (us)
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SysTickInit (uint32_t uiUsTime)
{
    SysTick->LOAD = 24 * uiUsTime - 1;                                  /* 定时时间设置                 */
    SysTick->VAL  = 0;                                                  /* SysTick计数值清零            */
    NVIC_EnableIRQ(SysTick_IRQn);                                       /* 打开SysTick中断              */
    
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;                            /* 使能系统节拍定时器并使能中断 */
}

/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define UART_BPS        115200                                          /* 串口通信波特率               */

/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        软件延时
** input parameters:    ulTime:延时时间(ms)
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void myDelay (uint32_t ulTime)
{
    uint32_t i;
    
    while (ulTime--) {
        for (i = 0; i < 2401; i++);
    }
}

/*********************************************************************************************************
** Function name:       UARTInit
** Descriptions:        UART初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void UARTInit (void)
{
    LPC_SWM->PINASSIGN[0] &= ~( 0xFFFF << 0 );
    LPC_SWM->PINASSIGN[0] |=  ( 4 << 0 );                               /* P0.4 ~ UART0_TXD             */
    LPC_SWM->PINASSIGN[0] |=  ( 0 << 8 );                               /* P0.0 ~ UART0_RXD             */
    
    LPC_SYSCON->UARTCLKDIV     = 1;                                     /* UART时钟分频值为 1           */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<14);                               /* 初始化UART AHB时钟           */
    
    LPC_USART0->BRG = SystemCoreClock * LPC_SYSCON->SYSAHBCLKDIV /
                      (LPC_SYSCON->UARTCLKDIV * 16 * UART_BPS) - 1;     /* 串口通信波特率               */
    
    LPC_USART0->CFG = (1 << 0) |                                        /* 使能UART                     */
                      (1 << 2) |                                        /* 8位数据位                    */
                      (0 << 4) |                                        /* 无校验                       */
                      (0 << 6);                                         /* 1位停止位                    */
}
    
/*********************************************************************************************************
** Function name:       UARTSendByte
** Descriptions:        UART发送字节，使用查询方式
** input parameters:    ucData:   要发送的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void UARTSendByte (uint8_t ucData)
{
    while ((LPC_USART0->STAT & 0x04) == 0);                             /* 等待发送数据准备好           */
    LPC_USART0->TXDATA = ucData;                                        /* 发送数据                     */
    while ((LPC_USART0->STAT & 0x08) == 0);                             /* 等待发送数据结束             */
}

/*********************************************************************************************************
** Function name:       UARTSendString
** Descriptions:        UART发送字符串
** input parameters:    pucData:   要发送的字符串指针 uiLen:  要发送的数据长度
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void UARTSendString (uint8_t *pucData, uint32_t uiLen)
{
    while (uiLen--) {
        UARTSendByte(*pucData++);
    }
}

/*********************************************************************************************************
** Function name:       UARTRecvByte
** Descriptions:        UART接收字节，使用查询方式
** input parameters:    无
** output parameters:   无
** Returned value:      ucRcvData:   接收到的数据
*********************************************************************************************************/
uint8_t UARTRecvByte (void)
{
    uint8_t ucData;
    
    while ((LPC_USART0->STAT & 0x01) == 0);                             /* 等待接收到数据               */
    ucData = LPC_USART0->RXDATA;                                        /* 读取数据                     */
    return (ucData);
}

/*********************************************************************************************************
** Function name:       UARTRecvString
** Descriptions:        串口接收字符串
** input parameters:    pucData:  要接收的字符串指针 uiLen:   要接收的数据长度
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void UARTRecvString (uint8_t *pucData, uint32_t uiLen)
{
    for (; uiLen > 0; uiLen--){
        *pucData++ = UARTRecvByte();
    }
}

/*********************************************************************************************************
** Function name:       main
** Descriptions:        UARTPolling 例程：
**                      串口参数：UART波特率115200、8个数据位、1个停止位、无奇偶校验位；
**                      串口连线：P0.0 连接 UART0_RXD   P0.4 连接 UART0_TXD
**                      操作方法：打开串口调试软件，运行程序，每次向下位机发送8字节数据，观察接收显示窗口
**                                串口调试软件显示下位机返回的数据。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
int main (void)
{
    uint8_t ucBuf[8];
    
    SystemInit();                                                       /* 初始化目标板，切勿删除       */
    UARTInit();                                                         /* UART初始化                   */
   // SysTickInit(1000);                                                /* SysTick 定时 10000us         */
    
    UARTSendString("UARTPolling Routine.\r\n", sizeof("UARTPolling Routine.\r\n"));
    
//    while (1) {
//        UARTRecvString(ucBuf, 8);                                       /* 从串口接收字符串             */
//        myDelay(1);
//        UARTSendString(ucBuf, 8);                                       /* 向串口发送字符串             */
//        myDelay(1);
//    }
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
