//
// main.cpp
// Main file
// ----------------------------------
// Developed with embedXcode
// http://embedXcode.weebly.com
//
// Project 		embdx_VM
//
// Created by 	Dehan Louw, 2015/06/08 12:41 PM
//              Dehan Louw
//
// Copyright 	© Dehan Louw, 2015
// Licence 		<#license#>
//
// See 			embdx_VM.ino and ReadMe.txt for references

//
// Last update: May 14, 2015 release 289

// IDE selection
#if defined(EMBEDXCODE)

// Core library and main()
#if defined(ESP8266)
// ============================================================================= ESP8266 specific

/*
 main.cpp - platform initialization and context switching
 emulation
 
 Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
 This file is part of the esp8266 core for Arduino environment.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//This may be used to change user task stack size:
//#define CONT_STACKSIZE 4096

#include <Arduino.h>
extern "C" {
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "cont.h"
}
#define LOOP_TASK_PRIORITY 0
#define LOOP_QUEUE_SIZE    1

int atexit(void (*func)()) { return 0; }

extern "C" void ets_update_cpu_frequency(int freqmhz);
void initVariant() __attribute__((weak));
void initVariant() { }

extern void loop();
extern void setup();

void preloop_update_frequency() __attribute__((weak));
void preloop_update_frequency()
{
#if defined(F_CPU) && (F_CPU == 16000000L)
    REG_SET_BIT(0x3ff00014, BIT(0));
    ets_update_cpu_frequency(160);
#endif
}



extern void (*__init_array_start)(void);
extern void (*__init_array_end)(void);

static cont_t g_cont;
static os_event_t g_loop_queue[LOOP_QUEUE_SIZE];

static uint32_t g_micros_at_task_start;

extern "C" uint32_t esp_micros_at_task_start()
{
    return g_micros_at_task_start;
}

extern "C" void abort()
{
    while(1){}
}

extern "C" void esp_yield()
{
    cont_yield(&g_cont);
}

extern "C" void esp_schedule()
{
    system_os_post(LOOP_TASK_PRIORITY, 0, 0);
}

extern "C" void __yield()
{
    esp_schedule();
    esp_yield();
}
extern "C" void yield(void) __attribute__ ((weak, alias("__yield")));


static void loop_wrapper()
{
    static bool setup_done = false;
    if (!setup_done)
    {
        setup();
        setup_done = true;
    }
    preloop_update_frequency();
    loop();
    esp_schedule();
}

static void loop_task(os_event_t *events)
{
    g_micros_at_task_start = system_get_time();
    cont_run(&g_cont, &loop_wrapper);
    if (cont_check(&g_cont) != 0)
    {
        ets_printf("\r\nheap collided with sketch stack\r\n");
        abort();
    }
}

static void do_global_ctors(void)
{
    void (**p)(void);
    for (p = &__init_array_start; p != &__init_array_end; ++p)
        (*p)();
}

void init_done()
{
    do_global_ctors();
    esp_schedule();
}

extern "C" {
    void user_init(void)
    {
        uart_div_modify(0, UART_CLK_FREQ / (115200));
        
        init();
        
        initVariant();
        
        cont_init(&g_cont);
        
        system_os_task( loop_task,
                       LOOP_TASK_PRIORITY,
                       g_loop_queue,
                       LOOP_QUEUE_SIZE);
        
        system_init_done_cb(&init_done);
    }
}


#elif defined(SPARK) || defined(PARTICLE)
// ============================================================================= Particle / Spark specific

/**
 ******************************************************************************
 * @file    main.cpp
 * @author  Satish Nair, Zachary Crockett, Zach Supalla and Mohit Bhoite
 * @version V1.0.0
 * @date    13-March-2013
 *
 * Updated: 14-Feb-2014 David Sidrane <david_s5@usa.net>
 *
 * @brief   Main program body.
 ******************************************************************************
 Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation, either
 version 3 of the License, or (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "spark_utilities.h"
#include "spark_wlan.h"
#include "core_hal.h"
#include "syshealth_hal.h"
#include "watchdog_hal.h"
#include "rgbled.h"

using namespace spark;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t TimingLED;
static volatile uint32_t TimingIWDGReload;

#ifdef MEASURE_LOOP_FREQUENCY
static volatile uint32_t loop_counter;
static volatile uint32_t loop_frequency;
#endif

/* Extern variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : HAL_SysTick_Handler (Declared as weak in core_hal.h)
 * Description    : Decrements the various Timing variables related to SysTick.
 * Input          : None
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
extern "C" void HAL_SysTick_Handler(void)
{
    if (LED_RGB_IsOverRidden())
    {
#ifndef SPARK_NO_CLOUD
        if ((LED_Spark_Signal != 0) && (NULL != LED_Signaling_Override))
        {
            LED_Signaling_Override();
        }
#endif
    }
    else if (TimingLED != 0x00)
    {
        TimingLED--;
    }
    else if(WLAN_SMART_CONFIG_START || SPARK_FLASH_UPDATE || Spark_Error_Count)
    {
        //Do nothing
    }
    else if(SPARK_LED_FADE)
    {
        LED_Fade(LED_RGB);
        TimingLED = 20;//Breathing frequency kept constant
    }
    else if(SPARK_CLOUD_CONNECTED)
    {
        LED_SetRGBColor(RGB_COLOR_CYAN);
        LED_On(LED_RGB);
        SPARK_LED_FADE = 1;
    }
    else
    {
        LED_Toggle(LED_RGB);
        if(SPARK_CLOUD_SOCKETED)
            TimingLED = 50;         //50ms
        else
            TimingLED = 100;        //100ms
    }
    
    if(SPARK_WLAN_SLEEP)
    {
        //Do nothing
    }
    else if(SPARK_FLASH_UPDATE)
    {
#ifndef SPARK_NO_CLOUD
        if (TimingFlashUpdateTimeout >= TIMING_FLASH_UPDATE_TIMEOUT)
        {
            //Reset is the only way now to recover from stuck OTA update
            HAL_Core_System_Reset();
        }
        else
        {
            TimingFlashUpdateTimeout++;
        }
#endif
    }
    else if(!WLAN_SMART_CONFIG_START && HAL_Core_Mode_Button_Pressed(3000))
    {
        //reset button debounce state if mode button is pressed for 3 seconds
        HAL_Core_Mode_Button_Reset();
        
        if(!SPARK_WLAN_SLEEP)
        {
            WLAN_SMART_CONFIG_START = 1;
        }
    }
    else if(HAL_Core_Mode_Button_Pressed(7000))
    {
        //reset button debounce state if mode button is pressed for 3+7=10 seconds
        HAL_Core_Mode_Button_Reset();
        
        WLAN_DELETE_PROFILES = 1;
    }
    
#ifdef IWDG_RESET_ENABLE
    if (TimingIWDGReload >= TIMING_IWDG_RELOAD)
    {
        TimingIWDGReload = 0;
        
        /* Reload WDG counter */
        HAL_Notify_WDT();
        DECLARE_SYS_HEALTH(CLEARED_WATCHDOG);
    }
    else
    {
        TimingIWDGReload++;
    }
#endif
}

/*******************************************************************************
 * Function Name  : HAL_RTC_Handler (Declared as weak in rtc_hal.h)
 * Description    : This function handles RTC global interrupt request.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
extern "C" void HAL_RTC_Handler(void)
{
#ifdef MEASURE_LOOP_FREQUENCY
    loop_frequency = loop_counter;
    loop_counter = 0;
#endif
    
    if(NULL != Time_Update_Handler)
    {
        Time_Update_Handler();
    }
}

/*******************************************************************************
 * Function Name  : HAL_RTCAlarm_Handler (Declared as weak in rtc_hal.h)
 * Description    : This function handles additional application requirements.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
extern "C" void HAL_RTCAlarm_Handler(void)
{
    /* Wake up from Spark.sleep mode(SLEEP_MODE_WLAN) */
    SPARK_WLAN_SLEEP = 0;
}

#ifdef MEASURE_LOOP_FREQUENCY
/* Utility call declared as weak - used to return loop() frequency measured in Hz */
uint32_t loop_frequency_hz()
{
    return loop_frequency;
}
#endif

/*******************************************************************************
 * Function Name  : main.
 * Description    : main routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void app_setup_and_loop(void)
{
    HAL_Core_Init();
    // We have running firmware, otherwise we wouldn't have gotten here
    DECLARE_SYS_HEALTH(ENTERED_Main);
    DEBUG("Hello from Spark!");
    
    SPARK_WLAN_Setup(Multicast_Presence_Announcement);
    
    /* Main loop */
    while (1)
    {
        DECLARE_SYS_HEALTH(ENTERED_WLAN_Loop);
        SPARK_WLAN_Loop();
        
        static uint8_t SPARK_WIRING_APPLICATION = 0;
        if(SPARK_WLAN_SLEEP || !SPARK_CLOUD_CONNECT || SPARK_CLOUD_CONNECTED || SPARK_WIRING_APPLICATION)
        {
            if(!SPARK_FLASH_UPDATE && !HAL_watchdog_reset_flagged())
            {
                if((SPARK_WIRING_APPLICATION != 1) && (NULL != setup))
                {
                    //Execute user application setup only once
                    DECLARE_SYS_HEALTH(ENTERED_Setup);
                    setup();
                    SPARK_WIRING_APPLICATION = 1;
                }
                
                if(NULL != loop)
                {
                    //Execute user application loop
                    DECLARE_SYS_HEALTH(ENTERED_Loop);
#ifdef MEASURE_LOOP_FREQUENCY
                    loop_counter++;
#endif
                    loop();
                    DECLARE_SYS_HEALTH(RAN_Loop);
                }
            }
        }
    }
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {
    }
}
#endif


#elif defined(MBED)
// ============================================================================= mbed specific

#include "mbed.h"

/**
 * Default setup function. This function may be overridden.
 */
void setup() __attribute__((weak));
//void setup() {}

/**
 * Default loop function. This function may be overridden.
 */
void loop() __attribute__((weak));
//void loop() {}

int main(void)
{
//    init();
    
    setup();
    
    for (;;)
    {
        loop();
    }
    return 0;
}


#elif defined(MPIDE)
// ============================================================================= chipKIT specific

//************************************************************************
//*	main.c
//*
//*	Arduino core files for PIC32
//*		Copyright (c) 2010, 2011 by Mark Sproul
//*
//*
//************************************************************************
//*	this code is based on code Copyright (c) 2005-2006 David A. Mellis
//*
//*	This library is free software; you can redistribute it and/or
//*	modify it under the terms of the GNU Lesser General Public
//*	License as published by the Free Software Foundation; either
//*	version 2.1 of the License, or (at your option) any later version.
//*
//*	This library is distributed in the hope that it will be useful,
//*	but WITHOUT ANY WARRANTY; without even the implied warranty of
//*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.//*	See the GNU
//*	Lesser General Public License for more details.
//*
//*	You should have received a copy of the GNU Lesser General
//*	Public License along with this library; if not, write to the
//*	Free Software Foundation, Inc., 59 Temple Place, Suite 330,
//*	Boston, MA  02111-1307  USA
//*
//*
//************************************************************************
//*	Edit History
//************************************************************************
//*	Oct 12,	2010	Got MPLAB X working on MacOSX 1.6 for the first time
//* Dec 12, 2011 <GeneApperson> added call to _scheduleTask() before call
//*					to loop().
//************************************************************************

#define OPT_SYSTEM_INTERNAL
#include <System_Defs.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

extern "C" {
    extern void __use_isr_install(void);
    __attribute__((section(".comment"))) void (*__use_force_isr_install)(void) = &__use_isr_install;
}

//************************************************************************
int main(void)
{
	init();
    
	setup();
    
	while (1)
	{
		_scheduleTask();
		loop();
	}
	return 0;
}


#elif defined(DIGISPARK)
// ============================================================================= Digispark specific

#include "WProgram.h"

int main(void)
{
	init();
    
	setup();
    
	for (;;)
		loop();
    
	return 0;
}



#elif defined(MICRODUINO)
// ============================================================================= Microduino specific

#include "Arduino.h"

int main(void)
{
	init();
    
#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
    
	return 0;
}


#elif defined(ENERGIA)
// ============================================================================= LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific

#if defined(__CC3200R1M1RGC__)
// ----------------------------------------------------------------------------- LaunchPad CC3200 specific

#include <Energia.h>
#include "inc/hw_gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/prcm.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include <driverlib/utils.h>
#include "inc/hw_hib1p2.h"
#include "inc/hw_hib3p3.h"

extern void (* const g_pfnVectors[])(void);

#ifdef __cplusplus
extern "C" {
#endif
    
    void _init(void)
    {
        IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
        
        MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
        
        MAP_IntMasterEnable();
        PRCMCC3200MCUInit();
        MAP_SysTickIntEnable();
        MAP_SysTickPeriodSet(F_CPU / 1000);
        MAP_SysTickEnable();
    }
    
#ifdef __cplusplus
} /* extern "C" */
#endif

int main(void)
{
    setup();
    
    for (;;) {
        loop();
        if (serialEventRun) serialEventRun();
    }
}


#elif defined(__LM4F120H5QR__) || defined(__TM4C1230C3PM__) || defined(__TM4C129XNCZAD__) || defined(__TM4C123GH6PM__)
// ----------------------------------------------------------------------------- LaunchPad Stellaris and Tiva specific
#include <Energia.h>

#if defined(PART_TM4C129XNCZAD)
#include "inc/tm4c129xnczad.h"
#elif defined(PART_TM4C1294NCPDT)
#include "inc/tm4c1294ncpdt.h"
#elif defined(PART_TM4C1233H6PM) || defined(PART_LM4F120H5QR)
#include "inc/tm4c123gh6pm.h"
#else
#error "**** No PART defined or unsupported PART ****"
#endif

#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

#ifdef __cplusplus
extern "C" {
    
    void _init(void)
    {
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
        if(ROM_EEPROMInit() == EEPROM_INIT_ERROR) {
            if(ROM_EEPROMInit() != EEPROM_INIT_ERROR)
                EEPROMMassErase();
        }
        
        timerInit();
        
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
#ifdef TARGET_IS_SNOWFLAKE_RA0
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOT);
#endif
        
        //Unlock and commit NMI pins PD7 and PF0
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0x4C4F434B;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x1;
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    } /* void _init(void) */
    
} /* extern "C" */
#endif

int main(void)
{
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
}


#elif defined(__MSP432P401R__)
// -----------------------------------------------------------------------------  LaunchPad MSP432 with RTOS specific

/*
 *  ======== main.cpp ========
 *  MT wiring Task framework
 */
#include <stddef.h>
//#include <oslib/osi.h>

/* XDC Header files */
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/Power.h>
/* Board Support Header files (from configuration closure) */
// Valid for Energia 15
#include "src/platform/Board.h"
// Valid for Energia 16
//#include "Board.h"
#include <Energia.h>

//#if defined(__TI_COMPILER_VERSION__) || defined(__GNUC__)
//__extern int __UNUSED_start__, __UNUSED_end__;
//#define START (&__UNUSED_start__)
//#define END   (&__UNUSED_end__)
//#else
//#define START NULL
//#define END   NULL
//#endif

/* magic insertion point 769d20fcd7a0eedaf64270f591438b01 */
#include "rtosTasks.h"


/*
 __extern void setup();
 __extern void loop();
 
 #define NUM_SKETCHES 1
 
 void (*func_ptr[NUM_SKETCHES][2])(void) = {
	{setup, loop}
 };
 */

xdc_Void the_task(xdc_UArg _task_setup, xdc_UArg _task_loop);

/* set priority of simple link callbacks
 * must be >= 0 and < Task_numPriorities
 * where Task_numPriorities is set by
 * TI-RTOS config
 */
//#define SIMPLELINK_PRI 3

/* Wiring-specific GPIO HW interrupt vectors */
__extern void Wiring_GPIO_hwiIntFxn(xdc_UArg callbacks);

/*
 *  ======== main task ========
 */
xdc_Void the_task(xdc_UArg _task_setup, xdc_UArg _task_loop)
{
    /* Call setup once */
    (*(void(*)()) _task_setup)();
    
    /* Call loop repeatedly */
    for(;;) {
        (*(void(*)()) _task_loop)();
        System_flush();
        Task_yield();
    }
}
/*
 *  ======== main ========
 */
int main()
{
    /* initialize all device/board specific peripherals */
    Board_init();  /* this function is generated as part of TI-RTOS config */
    Power_enablePolicy();
    
    //	System_printf("unused memory: start = %p, end = %p\n", START, END);
    
    /* The SimpleLink Host Driver requires a mechanism to allow functions to
     * execute in task context.  The SpawnTask is created to handle such
     * situations.  This task will remain blocked until the host driver
     * posts a function.  If the SpawnTask priority is higher than other
     * tasks, it will immediately execute that function and return to a
     * blocked state.  Otherwise, it will remain ready until it has
     * the highest priority of any ready function.
     */
    //	VStartSimpleLinkSpawnTask(SIMPLELINK_PRI);
    
    /* hijack the common hwi func to point to Wiring's handler that clears
     * the GPIO interrupt
     */
    
    //	Hwi_setFunc(Hwi_handle((Hwi_Struct *)Board_gpioCallbacks0.hwiStruct),
    //		Wiring_GPIO_hwiIntFxn, (UArg)&Board_gpioCallbacks0);
    //	Hwi_setFunc(Hwi_handle((Hwi_Struct *)Board_gpioCallbacks1.hwiStruct),
    //		Wiring_GPIO_hwiIntFxn, (UArg)&Board_gpioCallbacks1);
    //	Hwi_setFunc(Hwi_handle((Hwi_Struct *)Board_gpioCallbacks2.hwiStruct),
    //		Wiring_GPIO_hwiIntFxn, (UArg)&Board_gpioCallbacks2);
    //	Hwi_setFunc(Hwi_handle((Hwi_Struct *)Board_gpioCallbacks3.hwiStruct),
    //		Wiring_GPIO_hwiIntFxn, (UArg)&Board_gpioCallbacks3);
    
    Task_Params taskParams;
    
    System_printf("Startup\n");
    System_flush();
    
    /* initialize taskParams and set to default */
    Task_Params_init(&taskParams);
    
    /* All tasks have the same priority */
    taskParams.priority = Task_numPriorities - 1;
    taskParams.stackSize = 0x800;
    
    uint8_t i = 0;
    for(i = 0; i < NUM_SKETCHES; i++) {
        /* Set arg0 to setup() */
        taskParams.arg0 = (xdc_UArg)func_ptr[i][0];
        /* Set ar1 to loop */
        taskParams.arg1 = (xdc_UArg)func_ptr[i][1];
        /* Set the task name */
        taskParams.instance->name = (xdc_String) taskNames[i];
        /* Create the task */
        Task_create(the_task, &taskParams, NULL);
    }
    
    /* does not return */
    BIOS_start();
    
    return (0); /* should never get here, but just in case ... */
}

#else
// ----------------------------------------------------------------------------- LaunchPad MSP430 and Experimeter Board FR5739 specific

#include <Energia.h>

int main(void)
{
	init();
    
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
    
	return 0;
}

#endif                                                                          // Energia


#elif defined(MAPLE_IDE)
// ============================================================================= Maple specific

#include <WProgram.h>

// *****************************************************************************
//  The MIT License
//
//  Copyright (c) 2010 LeafLabs LLC.
//
//  Permission is hereby granted, free of charge, to any person
//  obtaining a copy of this software and associated documentation
//  files (the "Software"), to deal in the Software without
//  restriction, including without limitation the rights to use, copy,
//  modify, merge, publish, distribute, sublicense, and/or sell copies
//  of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
//  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
//  ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
// ****************************************************************************

__attribute__(( constructor )) void premain() {
    init();
}

int main(void) {
    setup();
    
    for (;;)
        loop();
    
    return 0;
}


#elif defined(TEENSYDUINO) || defined(CORE_TEENSY)
// ============================================================================= Teensy specific

#if defined(__MK20DX128__) || defined(__MK20DX256__)
// ----------------------------------------------------------------------------- Teensy 3 specific

#include "WProgram.h"

extern "C" int main(void)
{
	// Arduino's main() function just calls setup() and loop()....
	setup();
	while (1) {
		loop();
		yield();
	}
}

#else
// ----------------------------------------------------------------------------- Teensy 2 specific

#include "WProgram.h"

//int main(void) __attribute__((noreturn));
int main(void)
{
	_init_Teensyduino_internal_();
    
	setup();
    
    for (;;)
        loop();
}


#endif                                                                          // Teensy


#elif defined(PANSTAMP_AVR) || defined(PANSTAMP_NRG) || defined(ARDUINO_PANSTAMP_AVR)
// ============================================================================= panStamp specific

#include "Arduino.h"

int main(void)
{
    init();

    setup();
    
    for (;;) {
        loop();
        //if (serialEventRun) serialEventRun();
    }
    
    return 0;
}


#elif defined(WIRING)
// ============================================================================= Wiring specific

// Replace #include "WProgram.h" by #include "Wiring.h"
// Comment boardInit();

#include "Wiring.h"

int main(void)
{
    // Hardware specific initializations.
    //    boardInit();
    boardInit();
    
    // User defined setup routine
    setup();
    // User defined loop routine
    for(;;)
        loop();
}


#elif defined(REDBEARLAB)
// ============================================================================= RedBearLab specific

#define ARDUINO_MAIN

#include "Arduino.h"

/*
 * \brief Main entry point of Arduino application
 */


int main( void )
{
    lfclk_config();
    //RTC1_Init();
    rtc_timer_init();
    setup();
    
    for (;;)
    {
        loop();
    }
    
    return 0;
}


#elif defined(ARDUINO)
// ============================================================================= Arduino specific

#if (ARDUINO < 100)
// ----------------------------------------------------------------------------- Arduino 0023 specific

#include "WProgram.h"

int main(void)
{
	init();
    
	setup();
    
	for (;;)
		loop();
    
	return 0;
}


#elif (ARDUINO < 150)
// ----------------------------------------------------------------------------- Arduino 1.0.x specific

#include "Arduino.h"

int main(void)
{
	init();
    
#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
    
	return 0;
}


#elif (ARDUINO < 160)
// ----------------------------------------------------------------------------- Arduino 1.5.x specific

#if defined(__ARDUINO_X86__)
// ............................................................................. Arduino 1.5.x X86 architecture specific

/*
 main.cpp userspace main loop for Intel Galileo family boards
 Copyright (C) 2014 Intel Corporation
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 
 */
// Arduino hooks
#include <Arduino.h>
#include <trace.h>
#include <interrupt.h>
#include <sys/stat.h>

#define PLATFORM_NAME_PATH "/sys/devices/platform/"

/************************ Static *************************/
#define MY_TRACE_PREFIX __FILE__

/************************ Global *************************/
int main(int argc, char * argv[])
{
	char *platform_path = NULL;
	struct stat s;
	int err;
    
	// Install a signal handler
    
	// make ttyprintk at some point
	stdout = freopen("/tmp/log.txt", "w", stdout);
	if (stdout == NULL){
	    fprintf(stderr, "unable to remap stdout !\n");
	    exit(-1);
	}
	fflush(stdout);
    
	stderr = freopen("/tmp/log_er.txt", "w", stderr);
	if (stderr == NULL){
	    printf("Unable to remap stderr !\n");
	    exit(-1);
	}
	fflush(stderr);
    
	// Snapshot time counter
	if (timeInit() < 0)
		exit(-1);
    
	// debug for the user
	if (argc < 2){
		fprintf(stderr, "./sketch tty0\n");
		return -1;
	}
	printf("started with binary=%s Serial=%s\n", argv[0], argv[1]);
	fflush(stdout);
    
	// check if we're running on the correct platform
	// and refuse to run if no match
    
#ifdef GALILEO_IDE
	platform_path = (char *)malloc(sizeof(PLATFORM_NAME_PATH) + sizeof(PLATFORM_NAME));
	sprintf(platform_path,"%s%s", PLATFORM_NAME_PATH, PLATFORM_NAME);
    
	printf("checking platform_path [%s]\n", platform_path);
	fflush(stdout);
    
	err = stat(platform_path, &s);
    
	if(err != 0) {
		fprintf(stderr, "stat failed checking for %s with error code %d\n", PLATFORM_NAME, err);
		free(platform_path);
		return -1;
	}
	if(!S_ISDIR(s.st_mode)) {
		/* exists but is no dir */
		fprintf(stderr, "Target board not a %s\n", PLATFORM_NAME);
		free(platform_path);
		return -1;
	}
    
	printf("Running on a %s platform (%s)\n", PLATFORM_NAME, platform_path);
	fflush(stdout);
    
	free(platform_path);
#endif
    
	// TODO: derive trace level and optional IP from command line
	trace_init(VARIANT_TRACE_LEVEL, 0);
	trace_target_enable(TRACE_TARGET_UART);
    
	// Call Arduino init
	init(argc, argv);
    
	// Init IRQ layer
	// Called after init() to ensure I/O permissions inherited by pthread
	interrupt_init();
    
#if defined(USBCON)
	USBDevice.attach();
#endif
    
	setup();
	for (;;) {
		loop();
		//if (serialEventRun) serialEventRun();
	}
	return 0;
}


#elif defined(__SAM3X8E__)
// ............................................................................. Arduino 1.5.x SAM architecture specific

/*
 Copyright (c) 2011 Arduino.  All rights reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define ARDUINO_MAIN
#include "Arduino.h"

/*
 * Cortex-M3 Systick IT handler
 */
/*
 extern void SysTick_Handler( void )
 {
 // Increment tick count each ms
 TimeTick_Increment() ;
 }
 */

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	init();
    
	delay(1);
    
#if defined(USBCON)
	USBDevice.attach();
#endif
    
	setup();
    
	for (;;)
	{
		loop();
		if (serialEventRun) serialEventRun();
	}
    
	return 0;
}

#else
// ............................................................................. Arduino 1.5.x AVR architecture specific

#include "Arduino.h"

int main(void)
{
	init();
    
#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
    
	return 0;
}


#endif                                                                          // end architecture

#else
// ----------------------------------------------------------------------------- Arduino 1.6.x specific

#if defined(__ARDUINO_X86__)
// ............................................................................. Arduino 1.6.x X86 architecture specific

/*
 main.cpp userspace main loop for Intel Galileo family boards
 Copyright (C) 2014 Intel Corporation
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 
 */
// Arduino hooks
#include <Arduino.h>
#include <trace.h>
#include <interrupt.h>
#include <sys/stat.h>

#define PLATFORM_NAME_PATH "/sys/devices/platform/"

/************************ Static *************************/
#define MY_TRACE_PREFIX __FILE__

/************************ Global *************************/
int main(int argc, char * argv[])
{
    char *platform_path = NULL;
    struct stat s;
    int err;
    
    // Install a signal handler
    
    // make ttyprintk at some point
    stdout = freopen("/tmp/log.txt", "w", stdout);
    if (stdout == NULL){
        fprintf(stderr, "unable to remap stdout !\n");
        exit(-1);
    }
    fflush(stdout);
    
    stderr = freopen("/tmp/log_er.txt", "w", stderr);
    if (stderr == NULL){
        printf("Unable to remap stderr !\n");
        exit(-1);
    }
    fflush(stderr);
    
    // Snapshot time counter
    if (timeInit() < 0)
    exit(-1);
    
    // debug for the user
    if (argc < 2){
        fprintf(stderr, "./sketch tty0\n");
        return -1;
    }
    printf("started with binary=%s Serial=%s\n", argv[0], argv[1]);
    fflush(stdout);
    
    // check if we're running on the correct platform
    // and refuse to run if no match
    
#ifdef GALILEO_IDE
    platform_path = (char *)malloc(sizeof(PLATFORM_NAME_PATH) + sizeof(PLATFORM_NAME));
    sprintf(platform_path,"%s%s", PLATFORM_NAME_PATH, PLATFORM_NAME);
    
    printf("checking platform_path [%s]\n", platform_path);
    fflush(stdout);
    
    err = stat(platform_path, &s);
    
    if(err != 0) {
        fprintf(stderr, "stat failed checking for %s with error code %d\n", PLATFORM_NAME, err);
        free(platform_path);
        return -1;
    }
    if(!S_ISDIR(s.st_mode)) {
        /* exists but is no dir */
        fprintf(stderr, "Target board not a %s\n", PLATFORM_NAME);
        free(platform_path);
        return -1;
    }
    
    printf("Running on a %s platform (%s)\n", PLATFORM_NAME, platform_path);
    fflush(stdout);
    
    free(platform_path);
#endif
    
    // TODO: derive trace level and optional IP from command line
    trace_init(VARIANT_TRACE_LEVEL, 0);
    trace_target_enable(TRACE_TARGET_UART);
    
    // Call Arduino init
    init(argc, argv);
    
    // Init IRQ layer
    // Called after init() to ensure I/O permissions inherited by pthread
    interrupt_init();
    
#if defined(USBCON)
    USBDevice.attach();
#endif
    
    setup();
    for (;;) {
        loop();
        //if (serialEventRun) serialEventRun();
    }
    return 0;
}


#elif defined(__SAM3X8E__)
// ............................................................................. Arduino 1.6.x SAM architecture specific

/*
 main.cpp - Main loop for Arduino sketches
 Copyright (c) 2005-2013 Arduino Team.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define ARDUINO_MAIN
#include "Arduino.h"

/*
 main.cpp - Main loop for Arduino sketches
 Copyright (c) 2005-2013 Arduino Team.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define ARDUINO_MAIN
#include "Arduino.h"

/*
 * Cortex-M3 Systick IT handler
 */
/*
 extern void SysTick_Handler( void )
 {
 // Increment tick count each ms
 TimeTick_Increment() ;
 }
 */

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
    init();
    
    initVariant();
    
    delay(1);
    
#if defined(USBCON)
    USBDevice.attach();
#endif
    
    setup();
    
    for (;;)
    {
        loop();
        if (serialEventRun) serialEventRun();
    }
    
    return 0;
}

#elif defined(__SAMD21G18A__)

/*
 main.cpp - Main loop for Arduino sketches
 Copyright (c) 2005-2013 Arduino Team.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define ARDUINO_MAIN
#include "Arduino.h"

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
    init();
    
    delay(1);
#if defined(USBCON)
    USBDevice.init();
    USBDevice.attach();
#endif

    setup();
    
    for (;;)
    {
        loop();
        if (serialEventRun) serialEventRun();
    }
    
    return 0;
}


#else
// ............................................................................. Arduino 1.6.x AVR architecture specific


#include <Arduino.h>

//Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*func)()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

int main(void)
{
    init();
    
    initVariant();
    
#if defined(USBCON)
    USBDevice.attach();
#endif
    
    setup();
    
    for (;;) {
        loop();
        if (serialEventRun) serialEventRun();
    }
    
    return 0;
}

#endif                                                                          // end architecture

#endif                                                                          // end Arduino

#else                                                                           // error
#error Platform not defined
#endif                                                                          // end IDE

// Sketch
#include "embdx_VM.cpp"


#endif                                                                          // end embedXcode

