/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "board.h"
#include "fsl_lpadc.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_power.h"
#include "mcmgr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID (RL_PLATFORM_LPC55S69_M33_M33_LINK_ID)
#define MCMGR_USED

/* Address of RAM, where the image for core1 should be copied */
#define CORE1_BOOT_ADDRESS 0x20033000

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#elif (defined(__GNUC__)) && (!defined(__MCUXPRESSO))
extern const char m0_image_start[];
extern const char *m0_image_end;
extern uint32_t m0_image_size;
#define CORE1_IMAGE_START ((void *)m0_image_start)
#define CORE1_IMAGE_SIZE  ((void *)m0_image_size)
#endif
#define APP_TASK_STACK_SIZE        (256U)
#define LOCAL_EPT_ADDR             (40U)
#define APP_RPMSG_READY_EVENT_DATA (1U)

typedef struct the_message
{
    uint32_t DATA;
} THE_MESSAGE, *THE_MESSAGE_PTR;

static THE_MESSAGE msg = {0};
volatile uint32_t remote_addr = 0U;
struct rpmsg_lite_endpoint *my_ept;
rpmsg_queue_handle my_queue;
struct rpmsg_lite_instance *my_rpmsg;
rpmsg_ns_handle ns_handle;
uint32_t length;

#define SH_MEM_TOTAL_SIZE (6144U)
#ifdef MCMGR_USED
#if defined(__ICCARM__) /* IAR Workbench */
#pragma location = "rpmsg_sh_mem_section"
static char rpmsg_lite_base[SH_MEM_TOTAL_SIZE];
#elif defined(__GNUC__)
static char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section(".noinit.$rpmsg_sh_mem")));
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION) /* Keil MDK */
static char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section("rpmsg_sh_mem_section")));
#else
#error "RPMsg: Please provide your definition of rpmsg_lite_base[]!"
#endif
#endif /* MCMGR_USED */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif

static void receive_c0_task(void *pvParameters);
static void write_c0_task(void *pvParameters);
static void app_c0_task(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void)
{
    uint32_t core1_image_size;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    core1_image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__sec_core"
    core1_image_size = (uint32_t)__section_end("__sec_core") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__)
    core1_image_size = (uint32_t)m0_image_size;
#endif
    return core1_image_size;
}
#endif
static TaskHandle_t app_task_handle = NULL;

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
    uint32_t *data = (uint32_t *)user_data;

    *data = new_ept;
}

#ifdef MCMGR_USED
static volatile uint16_t RPMsgRemoteReadyEventData = 0U;
static void RPMsgRemoteReadyEventHandler(uint16_t eventData, void *context)
{
    uint16_t *data = (uint16_t *)context;

    *data = eventData;
}

uint8_t g_counter = 1U;

/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    (void)MCMGR_EarlyInit();
}
#endif /* MCMGR_USED */

SemaphoreHandle_t xMutex;

/*!
 * @brief Main function
 */
int main(void)
{
	xMutex1 = xSemaphoreCreateMutex();
    /* Initialize standard SDK demo application pins */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /*Board initialization*/
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Print the initial banner */
    (void)PRINTF("\r\nRPMsg demo starts\r\n");

    if (xTaskCreate(app_c0_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &app_task_handle) != pdPASS)
    {
        (void)PRINTF("\r\nFailed to create application task\r\n");
        for (;;)
        {
        }
    }

    vTaskStartScheduler();

    (void)PRINTF("Failed to start FreeRTOS on core0.\r\n");
    for (;;)
    {
    }
}

static void receive_core0_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex1, portMAX_DELAY) != pdTRUE)
        {
            PRINTF("Failed to take semaphore.\r\n");
        }
        (void)rpmsg_queue_recv(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char *)&msg, sizeof(THE_MESSAGE), &length,
                                       RL_BLOCK);
        (void)PRINTF("DATA = %i\r\n", msg.DATA);

        xSemaphoreGive(xMutex1);
        taskYIELD();
    }
}

static void app_core0_task(void *pvParameters)
{

#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* Calculate size of the image */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();
    (void)PRINTF("Copy CORE1 image to address: 0x%x, size: %d\r\n", (void *)(char *)CORE1_BOOT_ADDRESS,
                 core1_image_size);

    /* Copy application from FLASH to RAM */
    (void)memcpy((void *)(char *)CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();

    /* Register the application event before starting the secondary core */
    (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RPMsgRemoteReadyEventHandler,
                              (void *)&RPMsgRemoteReadyEventData);

    /* Boot Secondary core application */
    (void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, (uint32_t)rpmsg_lite_base,
                          kMCMGR_Start_Synchronous);

    /* Wait until the secondary core application signals the rpmsg remote has been initialized and is ready to
     * communicate. */
    while (APP_RPMSG_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
    {
    };

    my_rpmsg = rpmsg_lite_master_init(rpmsg_lite_base, SH_MEM_TOTAL_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
#else
    my_rpmsg =
        rpmsg_lite_master_init((void *)RPMSG_LITE_SHMEM_BASE, SH_MEM_TOTAL_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
#endif
    my_queue  = rpmsg_queue_create(my_rpmsg);
    my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, (void *)&remote_addr);

    /* Wait until the secondary core application issues the nameservice isr and the remote endpoint address is known. */
    while (0U == remote_addr)
    {
    };

    /* Send the first message to the remoteproc */
    msg.DATA = g_counter;
    (void)rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);

    if (xTaskCreate(receive_core0_task, "RECEIVE_C0_TASK", configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 1, NULL) !=
    	pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}
	if (xTaskCreate(write_core0_task, "WRITE_Core0_TASK", configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 1, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}
	vTaskSuspend(NULL);

    /* Print the ending banner */
    (void)PRINTF("\r\nRPMsg demo ends\r\n");
    for (;;)
    {
    }
}



static void write_core0_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex1, portMAX_DELAY) != pdTRUE)
        {
            PRINTF("Failed to take semaphore.\r\n");
        }

        g_counter++;
		msg.DATA = g_counter;

		if(g_counter <= 100){
			(void)rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&msg, sizeof(THE_MESSAGE), RL_BLOCK);
			xSemaphoreGive(xMutex1);
			taskYIELD();
		}
		else{
			(void)rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
			my_ept = ((void *)0);
			(void)rpmsg_queue_destroy(my_rpmsg, my_queue);
			my_queue = ((void *)0);
			(void)rpmsg_ns_unbind(my_rpmsg, ns_handle);
			(void)rpmsg_lite_deinit(my_rpmsg);
			vTaskEndScheduler();
		}


    }
}
