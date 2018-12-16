/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the 
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt 
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt 
 * timing.  The maximum measured jitter time is latched in the usMaxJitter 
 * variable, and displayed on the LCD by the 'Check' as described below.  
 * The fast interrupt is configured and handled in the timer_test.c source 
 * file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the LCD directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of 
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting 
 * for messages - waking and displaying the messages as they arrive.  The LCD
 * task is defined in lcd.c.  
 * 
 * "Check" task -  This only executes every three seconds but has the highest 
 * priority so is guaranteed to get processor time.  Its main function is to 
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write "FAIL #n" to the LCD (via the LCD task).  If all the demo tasks are 
 * executing with their expected behaviour then the check task writes the max
 * jitter time to the LCD (again via the LCD task), as described above.
 */

// PIC24FJ64GA306 Configuration Bit Settings

// 'C' source line config statements

// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits (1:68719476736 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select (DSWDT uses LPRC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep BOR Enable bit (DSBOR Enabled)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable (DSWDT Enabled)
#pragma config DSSWEN = ON              // DSEN Bit Enable (Deep Sleep is controlled by the register bit DSEN)

// CONFIG3
#pragma config WPFP = WPFP63            // Write Protection Flash Page Segment Boundary (Page 52 (0xFC00))
#pragma config VBTBOR = ON              // VBAT BOR enable bit (VBAT BOR enabled)
#pragma config SOSCSEL = ON             // SOSC Selection bits (SOSC circuit selected)
#pragma config WDTWIN = PS25_0          // Watch Dog Timer Window Width (Watch Dog Timer Window Width is 25 percent)
#pragma config BOREN = ON               // Brown-out Reset Enable (Brown-out Reset Enable)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Disabled)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMD = NONE            // Primary Oscillator Select (Primary Oscillator Disabled)
#pragma config BOREN1 = EN              // BOR Override bit (BOR Enabled [When BOREN=1])
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Once set, the IOLOCK bit cannot be cleared)
#pragma config OSCIOFCN = OFF           // OSCO Pin Configuration (OSCO/CLKO/RC15 functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Fail-Safe Clock Monitor Configuration bits (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRCDIV           // Initial Oscillator Select (Fast RC Oscillator with Postscaler (FRCDIV))
#pragma config ALTVREF = DLT_AV_DLT_CV  // Alternate VREF/CVREF Pins Selection bit (Voltage reference input, ADC =RA9/RA10 Comparator =RA9,RA10)
#pragma config IESO = ON                // Internal External Switchover (Enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler Select (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler Ratio Select (1:128)
#pragma config FWDTEN = WDT_HW          // Watchdog Timer Enable (WDT enabled in hardware)
#pragma config WINDIS = OFF             // Windowed WDT Disable (Standard Watchdog Timer)
#pragma config ICS = PGx2               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC2/PGED2)
#pragma config LPCFG = OFF              // Low power regulator control (Disabled)
#pragma config GWRP = OFF               // General Segment Write Protect (Disabled)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = ON              // JTAG Port Enable (Enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

/* Demo application includes. */
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"
#include "lcd.h"
#include "timertest.h"

/* Demo task priorities. */
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY				( 2 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE				( 19200 )

/* The LED used by the comtest tasks.  mainCOM_TEST_LED + 1 is also used.
See the comtest.c file for more information. */
#define mainCOM_TEST_LED					( 6 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )
#define START_STK_SIZE 128
#define START_TASK_PRIO   3
#define TASK1_STK_SIZE 128
#define TASK1_TASK_PRIO   4
#define TASK2_STK_SIZE 128
#define TASK2_TASK_PRIO   5
/*-----------------------------------------------------------*/

/*
 * The check task as described at the top of this file.
 */
//static void vCheckTask(void *pvParameters);

/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware(void);

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void task1_task(void *pvParameters);
void task2_task(void *pvParameters);
void start_task(void *pvParameters);
TaskHandle_t StartTask_Handle;
TaskHandle_t Task1Task_Handle;
TaskHandle_t Task2Task_Handle;
/*-----------------------------------------------------------*/

/* The queue used to send messages to the LCD task. */
//static QueueHandle_t xLCDQueue;

/*-----------------------------------------------------------*/

/*
 * Create the demo tasks then start the scheduler.
 */
int main(void) {
    /* Configure any hardware required for this demo. */
    prvSetupHardware();

    /* Create the standard demo tasks. */
    //    vStartBlockingQueueTasks(mainBLOCK_Q_PRIORITY);
    //    vStartIntegerMathTasks(tskIDLE_PRIORITY);
    //    vStartFlashCoRoutines(mainNUM_FLASH_COROUTINES);
    //    vAltStartComTestTasks(mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED);
    //    vCreateBlockTimeTasks();

    /* Create the test tasks defined within this file. */
    //    xTaskCreate(vCheckTask, "Check", mainCHECK_TAKS_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL);

    /* Start the task that will control the LCD.  This returns the handle
    to the queue used to write text out to the task. */
    //	xLCDQueue = xStartLCDTask();

    /* Start the high frequency interrupt test. */
    //    vSetupTimerTest(mainTEST_INTERRUPT_FREQUENCY);
    xTaskCreate((TaskFunction_t) start_task,
            (const char * const) "start", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            (const configSTACK_DEPTH_TYPE) START_STK_SIZE,
            (void * const) NULL,
            (UBaseType_t) START_TASK_PRIO,
            (TaskHandle_t * const) &StartTask_Handle);
    /* Finally start the scheduler. */
    vTaskStartScheduler();

    /* Will only reach here if there is insufficient heap available to start
    the scheduler. */
    return 0;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void) {
    vParTestInitialise();
}

/*-----------------------------------------------------------*/
//static void vCheckTask(void *pvParameters) {
//    /* Used to wake the task at the correct frequency. */
//    TickType_t xLastExecutionTime;
//
//    /* The maximum jitter time measured by the fast interrupt test. */
//    extern unsigned short usMaxJitter;
//
//    /* Buffer into which the maximum jitter time is written as a string. */
//    static char cStringBuffer[ mainMAX_STRING_LENGTH ];
//
//    /* The message that is sent on the queue to the LCD task.  The first
//    parameter is the minimum time (in ticks) that the message should be
//    left on the LCD without being overwritten.  The second parameter is a pointer
//    to the message to display itself. */
//    xLCDMessage xMessage = {0, cStringBuffer};
//
//    /* Set to pdTRUE should an error be detected in any of the standard demo tasks. */
//    unsigned short usErrorDetected = pdFALSE;
//
//    /* Remove compiler warnings. */
//    (void) pvParameters;
//
//    /* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
//    works correctly. */
//    xLastExecutionTime = xTaskGetTickCount();
//
//    for (;;) {
//        /* Wait until it is time for the next cycle. */
//        vTaskDelayUntil(&xLastExecutionTime, mainCHECK_TASK_PERIOD);
//
//        /* Has an error been found in any of the standard demo tasks? */
//
//        if (xAreIntegerMathsTaskStillRunning() != pdTRUE) {
//            usErrorDetected = pdTRUE;
//            sprintf(cStringBuffer, "FAIL #1");
//        }
//
//        if (xAreComTestTasksStillRunning() != pdTRUE) {
//            usErrorDetected = pdTRUE;
//            sprintf(cStringBuffer, "FAIL #2");
//        }
//
//        if (xAreBlockTimeTestTasksStillRunning() != pdTRUE) {
//            usErrorDetected = pdTRUE;
//            sprintf(cStringBuffer, "FAIL #3");
//        }
//
//        if (xAreBlockingQueuesStillRunning() != pdTRUE) {
//            usErrorDetected = pdTRUE;
//            sprintf(cStringBuffer, "FAIL #4");
//        }
//
//        if (usErrorDetected == pdFALSE) {
//            /* No errors have been discovered, so display the maximum jitter
//            timer discovered by the "fast interrupt test". */
//            sprintf(cStringBuffer, "%dns max jitter", (short) (usMaxJitter - mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS) * mainNS_PER_CLOCK);
//        }
//
//        /* Send the message to the LCD gatekeeper for display. */
//        //        xQueueSend(xLCDQueue, &xMessage, portMAX_DELAY);
//    }
//}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
    /* Schedule the co-routines from within the idle task hook. */
    //    vCoRoutineSchedule();
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void start_task(void *pvParameters) {
    xTaskCreate((TaskFunction_t) task1_task,
            (const char * const) "task1_task", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            (const configSTACK_DEPTH_TYPE) TASK1_STK_SIZE,
            (void * const) NULL,
            (UBaseType_t) TASK1_TASK_PRIO,
            (TaskHandle_t * const) &Task1Task_Handle);
    xTaskCreate((TaskFunction_t) task2_task,
            (const char * const) "task2_task", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            (const configSTACK_DEPTH_TYPE) TASK2_STK_SIZE,
            (void * const) NULL,
            (UBaseType_t) TASK2_TASK_PRIO,
            (TaskHandle_t * const) &Task2Task_Handle);
    vTaskDelete(StartTask_Handle); //删除自身使用NULL
}

void task1_task(void *pvParameters) {
    while (1) {
        _LATE5 ^= 1;
        vTaskDelay(50);
    }
}

void task2_task(void *pvParameters) {//red led
    while (1) {
        _LATE7 ^= 1;
        vTaskDelay(10);
    }
}

//void __attribute__((__interrupt__, auto_psv)) _DefaultInterrupt(void) {
//    uint16_t tmp;
//    Nop();
//    Nop();
//    Nop();
//    tmp = RCON;
//    Nop();
//    Nop();
//}

