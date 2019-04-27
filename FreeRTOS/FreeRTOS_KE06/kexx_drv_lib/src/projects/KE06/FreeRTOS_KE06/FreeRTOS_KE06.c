
/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2013 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file FreeRTOS_KE06.c
*
* @author Freescale
*
* @version 0.0.1
*
* @date Oct. 15, 2013
*
* @brief providing framework of demo cases for MCU. 
*
*******************************************************************************/

#include "common.h"
#include "sysinit.h"

#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************
* Global variables
******************************************************************************/
uint32_t SystemCoreClock = 40000000;



/* 开始任务句柄 */
xTaskHandle     m_start_task_handle;
/* LED任务 */
xTaskHandle     m_led_task_handle;
/******************************************************************************
* Constants and macros
******************************************************************************/
#define START_TASK_PRIO         1
#define START_STK_SIZE          128


#define LED_TASK_PRIO         2
#define LED_STK_SIZE          128
/******************************************************************************
* Local types
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/
void start_task(void *);
void led_task(void *);

/******************************************************************************
* Global functions
******************************************************************************/
int main (void)
{
    /* Perform processor initialization */
    sysinit();
    printf("\nRunning the KE06 FreeRTOS_KE06 project.\n");
    
    LED0_Init();        //red led
    LED0_Off();
    
    xTaskCreate(start_task,
                "start_task",
                START_STK_SIZE,
                NULL,
                START_TASK_PRIO,
                &m_start_task_handle);
    
    vTaskStartScheduler();

}




void start_task(void *p)
{
    taskENTER_CRITICAL();
    
    xTaskCreate(led_task,
                "led_task",
                LED_STK_SIZE,
                NULL,
                LED_TASK_PRIO,
                &m_led_task_handle);


    vTaskDelete(NULL);
    
    taskEXIT_CRITICAL();
}



void led_task(void *p)
{
    while(1) {
        LED0_Toggle();
        vTaskDelay(1000);
    }
}



