/*********************************************Copyright (c)***********************************************
**                                Guangzhou ZLG MCU Technology Co., Ltd.
**
**                                        http://www.zlgmcu.com
**
**      广州周立功单片机科技有限公司所提供的所有服务内容旨在协助客户加速产品的研发进度，在服务过程中所提供
**  的任何程序、文档、测试结果、方案、支持等资料和信息，都仅供参考，客户有权不使用或自行参考修改，本公司不
**  提供任何的完整性、可靠性等保证，若在客户使用过程中因任何原因造成的特别的、偶然的或间接的损失，本公司不
**  承担任何责任。
**                                                                        ——广州周立功单片机科技有限公司
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               zlg_stdint.h
** Last modified date:      2014-03-12
** Last version:            V1.0
** Description:             类型定义
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Li Xiang
** Created date:            2014-03-12
** Version:                 V1.0
** Descriptions:            The original version 初始版本
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#ifndef  __ZLG_STDINT_H
#define  __ZLG_STDINT_H

#ifdef __cplusplus
extern "C" {
#endif


/*********************************************************************************************************
  通用数据类型重定义
*********************************************************************************************************/
typedef unsigned char       BOOLEAN;                                    /* 布尔变量                     */
typedef unsigned char       INT8U;                                      /* 无符号8位整型变量            */
typedef signed   char       INT8S;                                      /* 有符号8位整型变量            */
typedef unsigned short      INT16U;                                     /* 无符号16位整型变量           */
typedef signed   short      INT16S;                                     /* 有符号16位整型变量           */
typedef unsigned long       INT32U;                                     /* 无符号32位整型变量           */
typedef signed   long       INT32S;                                     /* 有符号32位整型变量           */
typedef unsigned long long  INT64U;                                     /* 无符号64位整型变量           */
typedef signed long long    INT64S;                                     /* 有符号64位整型变量           */
typedef float               FP32;                                       /* 单精度浮点数（32位长度）     */
typedef double              FP64;                                       /* 双精度浮点数（64位长度）     */

typedef unsigned char       uint8;                                      /* 无符号8位整型变量            */
typedef unsigned short int  uint16;                                     /* 无符号16位整型变量           */
typedef unsigned long int   uint32;                                     /* 无符号32位整型变量           */

typedef char                int8;                                       /* 有符号8位整型变量            */
typedef short int           int16;                                      /* 有符号16位整型变量           */
typedef int                 int32;                                      /* 有符号32位整型变量           */

typedef volatile int8       vint8;                                      /*  8 bits                      */
typedef volatile int16      vint16;                                     /* 16 bits                      */
typedef volatile int32      vint32;                                     /* 32 bits                      */

typedef volatile uint8      vuint8;                                     /*  8 bits                      */
typedef volatile uint16     vuint16;                                    /* 16 bits                      */
typedef volatile uint32     vuint32;                                    /* 32 bits                      */


#ifdef __cplusplus
}
#endif

#endif


/*********************************************************************************************************
  END FILE 
*********************************************************************************************************/
