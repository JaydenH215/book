/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Technology Co., LTD
**
**                                 http://www.zlgmcu.com
**
**      ������������Ƭ���Ƽ����޹�˾���ṩ�����з�������ּ��Э���ͻ����ٲ�Ʒ���з����ȣ��ڷ�����������ṩ
**  ���κγ����ĵ������Խ����������֧�ֵ����Ϻ���Ϣ���������ο����ͻ���Ȩ��ʹ�û����вο��޸ģ�����˾��
**  �ṩ�κε������ԡ��ɿ��Եȱ�֤�����ڿͻ�ʹ�ù��������κ�ԭ����ɵ��ر�ġ�żȻ�Ļ��ӵ���ʧ������˾��
**  �е��κ����Ρ�
**                                                                        ����������������Ƭ���Ƽ����޹�˾
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
** Descriptions:        ����ģ�壬����û�Ӧ�ó���
**
*********************************************************************************************************/
#include "LPC8xx.h"
///*********************************************************************************************************
//** Function name:       SysTick_Handler
//** Descriptions:        ϵͳ���Ķ�ʱ���жϷ�����
//** input parameters:    ��
//** output parameters:   ��
//** Returned value:      ��
//*********************************************************************************************************/
//extern void xPortSysTickHandler(void);
//void SysTick_Handler (void)
//{
//    xPortSysTickHandler();
//    //LPC_GPIO_PORT->NOT[0] |= LED;                                       /* SysTick�жϣ�LED״̬��ת     */
//}
/*********************************************************************************************************
** Function name:       SysTickInit
** Descriptions:        ϵͳ���Ķ�ʱ����ʼ��
**                      uiUsTime����ʱ uiUsTime (us)
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void SysTickInit (uint32_t uiUsTime)
{
    SysTick->LOAD = 24 * uiUsTime - 1;                                  /* ��ʱʱ������                 */
    SysTick->VAL  = 0;                                                  /* SysTick����ֵ����            */
    NVIC_EnableIRQ(SysTick_IRQn);                                       /* ��SysTick�ж�              */
    
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;                            /* ʹ��ϵͳ���Ķ�ʱ����ʹ���ж� */
}

/*********************************************************************************************************
  �궨��
*********************************************************************************************************/
#define UART_BPS        115200                                          /* ����ͨ�Ų�����               */

/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        �����ʱ
** input parameters:    ulTime:��ʱʱ��(ms)
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        UART��ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void UARTInit (void)
{
    LPC_SWM->PINASSIGN[0] &= ~( 0xFFFF << 0 );
    LPC_SWM->PINASSIGN[0] |=  ( 4 << 0 );                               /* P0.4 ~ UART0_TXD             */
    LPC_SWM->PINASSIGN[0] |=  ( 0 << 8 );                               /* P0.0 ~ UART0_RXD             */
    
    LPC_SYSCON->UARTCLKDIV     = 1;                                     /* UARTʱ�ӷ�ƵֵΪ 1           */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<14);                               /* ��ʼ��UART AHBʱ��           */
    
    LPC_USART0->BRG = SystemCoreClock * LPC_SYSCON->SYSAHBCLKDIV /
                      (LPC_SYSCON->UARTCLKDIV * 16 * UART_BPS) - 1;     /* ����ͨ�Ų�����               */
    
    LPC_USART0->CFG = (1 << 0) |                                        /* ʹ��UART                     */
                      (1 << 2) |                                        /* 8λ����λ                    */
                      (0 << 4) |                                        /* ��У��                       */
                      (0 << 6);                                         /* 1λֹͣλ                    */
}
    
/*********************************************************************************************************
** Function name:       UARTSendByte
** Descriptions:        UART�����ֽڣ�ʹ�ò�ѯ��ʽ
** input parameters:    ucData:   Ҫ���͵�����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void UARTSendByte (uint8_t ucData)
{
    while ((LPC_USART0->STAT & 0x04) == 0);                             /* �ȴ���������׼����           */
    LPC_USART0->TXDATA = ucData;                                        /* ��������                     */
    while ((LPC_USART0->STAT & 0x08) == 0);                             /* �ȴ��������ݽ���             */
}

/*********************************************************************************************************
** Function name:       UARTSendString
** Descriptions:        UART�����ַ���
** input parameters:    pucData:   Ҫ���͵��ַ���ָ�� uiLen:  Ҫ���͵����ݳ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void UARTSendString (uint8_t *pucData, uint32_t uiLen)
{
    while (uiLen--) {
        UARTSendByte(*pucData++);
    }
}

/*********************************************************************************************************
** Function name:       UARTRecvByte
** Descriptions:        UART�����ֽڣ�ʹ�ò�ѯ��ʽ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ucRcvData:   ���յ�������
*********************************************************************************************************/
uint8_t UARTRecvByte (void)
{
    uint8_t ucData;
    
    while ((LPC_USART0->STAT & 0x01) == 0);                             /* �ȴ����յ�����               */
    ucData = LPC_USART0->RXDATA;                                        /* ��ȡ����                     */
    return (ucData);
}

/*********************************************************************************************************
** Function name:       UARTRecvString
** Descriptions:        ���ڽ����ַ���
** input parameters:    pucData:  Ҫ���յ��ַ���ָ�� uiLen:   Ҫ���յ����ݳ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void UARTRecvString (uint8_t *pucData, uint32_t uiLen)
{
    for (; uiLen > 0; uiLen--){
        *pucData++ = UARTRecvByte();
    }
}

/*********************************************************************************************************
** Function name:       main
** Descriptions:        UARTPolling ���̣�
**                      ���ڲ�����UART������115200��8������λ��1��ֹͣλ������żУ��λ��
**                      �������ߣ�P0.0 ���� UART0_RXD   P0.4 ���� UART0_TXD
**                      �����������򿪴��ڵ�����������г���ÿ������λ������8�ֽ����ݣ��۲������ʾ����
**                                ���ڵ��������ʾ��λ�����ص����ݡ�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
int main (void)
{
    uint8_t ucBuf[8];
    
    SystemInit();                                                       /* ��ʼ��Ŀ��壬����ɾ��       */
    UARTInit();                                                         /* UART��ʼ��                   */
   // SysTickInit(1000);                                                /* SysTick ��ʱ 10000us         */
    
    UARTSendString("UARTPolling Routine.\r\n", sizeof("UARTPolling Routine.\r\n"));
    
//    while (1) {
//        UARTRecvString(ucBuf, 8);                                       /* �Ӵ��ڽ����ַ���             */
//        myDelay(1);
//        UARTSendString(ucBuf, 8);                                       /* �򴮿ڷ����ַ���             */
//        myDelay(1);
//    }
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
