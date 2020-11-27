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

#include "fsl_lpadc.h"
#include "clock_config.h"
#include "fsl_inputmux.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_power.h"
#include "mcmgr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_LPC55S69_M33_M33_LINK_ID)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-openamp-demo-channel"
#define MCMGR_USED
#define APP_TASK_STACK_SIZE (256U)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30U)
#endif
#define APP_RPMSG_READY_EVENT_DATA (1U)

/*******************************************************************************
 * ADC Definitions
 ******************************************************************************/
#define DEMO_LPADC_BASE                  ADC0
#define DEMO_LPADC_IRQn                  ADC0_IRQn
#define DEMO_LPADC_IRQ_HANDLER_FUNC      ADC0_IRQHandler
#define DEMO_LPADC_TEMP_SENS_CHANNEL     26U
#define DEMO_LPADC_USER_CMDID            1U /* CMD1 */
#define DEMO_LPADC_VREF_SOURCE           kLPADC_ReferenceVoltageAlt2
#define DEMO_LPADC_DO_OFFSET_CALIBRATION false
#define DEMO_LPADC_OFFSET_VALUE_A        0x10U
#define DEMO_LPADC_OFFSET_VALUE_B        0x10U


volatile uint32_t remote_addr;
struct rpmsg_lite_endpoint *volatile my_ept;
volatile rpmsg_queue_handle my_queue;
struct rpmsg_lite_instance *volatile my_rpmsg;
volatile rpmsg_ns_handle ns_handle;

SemaphoreHandle_t xMutex;

typedef struct the_message
{
    int32_t DATA;
} THE_MESSAGE, *THE_MESSAGE_PTR;

static volatile THE_MESSAGE msg = {0};
#ifdef RPMSG_LITE_MASTER_IS_LINUX
static char helloMsg[13];
#endif /* RPMSG_LITE_MASTER_IS_LINUX */

/*******************************************************************************
 * Code
 ******************************************************************************/
static TaskHandle_t app_task_handle = NULL;

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}

#ifdef MCMGR_USED
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





/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ADC_Configuration(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/

lpadc_conv_command_config_t g_LpadcCommandConfigStruct; /* Structure to configure conversion command. */
volatile bool g_LpadcConversionCompletedFlag = false;
float g_CurrentTemperature                   = 0.0f;
const uint32_t g_LpadcFullRange              = 65536U;

/*******************************************************************************
 * Code
 ******************************************************************************/
void DEMO_LPADC_IRQ_HANDLER_FUNC(void)
{
    g_CurrentTemperature           = LPADC_MeasureTemperature(DEMO_LPADC_BASE, DEMO_LPADC_USER_CMDID, 0U);
    g_LpadcConversionCompletedFlag = true;
    SDK_ISR_EXIT_BARRIER;
}



/*!
 * @brief Main function
 */

static void receive_task(void *pvParameters);
static void write_task(void *pvParameters);
static void app_task(void *pvParameters);

int main(void)
{
	xMutex = xSemaphoreCreateMutex();

    /* Initialize standard SDK demo application pins */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);

    BOARD_InitBootPins();
    BOARD_InitDebugConsole_Core1();

    CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 0U, true);  /*!< Reset ADCCLKDIV divider counter and halt it */
	CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 2U, false); /*!< Set ADCCLKDIV divider to value 4 */

	CLOCK_AttachClk(kFRO_HF_to_ADC_CLK);

	/* Disable LDOGPADC power down */
	POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);
	/* Disable Temperature sensor power down. */
	POWER_DisablePD(kPDRUNCFG_PD_TEMPSENS);

	ADC_Configuration();

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();
#endif /* MCMGR_USED */

    if (xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &app_task_handle) != pdPASS)
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

static void app_task(void *pvParameters)
{

    /* Print the initial banner */
    (void)PRINTF("\r\nRPMSG Ping-Pong FreeRTOS RTOS API Demo...\r\n");

#ifdef MCMGR_USED
    uint32_t startupData;
    mcmgr_status_t status;

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);

    my_rpmsg = rpmsg_lite_remote_init((void *)(char *)startupData, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    /* Signal the other core we are ready by triggering the event and passing the APP_RPMSG_READY_EVENT_DATA */
    (void)MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, APP_RPMSG_READY_EVENT_DATA);
#else
    (void)PRINTF("RPMSG Share Base Addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
#endif /* MCMGR_USED */
    while (0 == rpmsg_lite_is_link_up(my_rpmsg))
    {
    }
    (void)PRINTF("Link is up!\r\n");

    my_queue  = rpmsg_queue_create(my_rpmsg);
    my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, ((void *)0));
    SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
    (void)PRINTF("Nameservice announce sent.\r\n");

#ifdef RPMSG_LITE_MASTER_IS_LINUX
    /* Wait Hello handshake message from Remote Core. */
    (void)rpmsg_queue_recv(my_rpmsg, my_queue, (uint32_t *)&remote_addr, helloMsg, sizeof(helloMsg), ((void *)0),
                           RL_BLOCK);
#endif /* RPMSG_LITE_MASTER_IS_LINUX */

    if (xTaskCreate(receive_task, "RECEIVE_TASK_1", configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 1, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}
	if (xTaskCreate(write_task, "WRITE_TASK", configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 1, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}
	vTaskSuspend(NULL);
}

static void receive_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE)
        {
            PRINTF("Failed to take semaphore.\r\n");
        }
        (void)rpmsg_queue_recv(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char *)&msg, sizeof(THE_MESSAGE),
                                       ((void *)0), RL_BLOCK);
        xSemaphoreGive(xMutex);
        taskYIELD();
    }
}


static void write_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE)
        {
            PRINTF("Failed to take semaphore.\r\n");
        }

        LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* 1U is trigger0 mask. */
		while (false == g_LpadcConversionCompletedFlag)
		{
		}

        if((msg.DATA%5) == 0){
        	msg.DATA = g_CurrentTemperature;
        }
        else{
        	msg.DATA = 0;
        }

        (void)rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&msg, sizeof(THE_MESSAGE), RL_BLOCK);

        xSemaphoreGive(xMutex);
        taskYIELD();
    }
}
 /*lapds Function*/
static void ADC_Configuration(void)
{
    lpadc_config_t lpadcConfigStruct;
    lpadc_conv_trigger_config_t lpadcTriggerConfigStruct;

    /* Init ADC peripheral. */
    LPADC_GetDefaultConfig(&lpadcConfigStruct);
    lpadcConfigStruct.enableAnalogPreliminary = true;
    lpadcConfigStruct.powerLevelMode          = kLPADC_PowerLevelAlt4;
#if defined(DEMO_LPADC_VREF_SOURCE)
    lpadcConfigStruct.referenceVoltageSource = DEMO_LPADC_VREF_SOURCE;
#endif /* DEMO_LPADC_VREF_SOURCE */
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS
    lpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;
#endif /* FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS */
    lpadcConfigStruct.FIFO0Watermark = FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE - 1U;
    LPADC_Init(DEMO_LPADC_BASE, &lpadcConfigStruct);
    LPADC_DoResetFIFO0(DEMO_LPADC_BASE);

    /* Do ADC calibration. */
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFS) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFS
#if defined(FSL_FEATURE_LPADC_HAS_OFSTRIM) && FSL_FEATURE_LPADC_HAS_OFSTRIM
    /* Request offset calibration. */
#if defined(DEMO_LPADC_DO_OFFSET_CALIBRATION) && DEMO_LPADC_DO_OFFSET_CALIBRATION
    LPADC_DoOffsetCalibration(DEMO_LPADC_BASE);
#else
    LPADC_SetOffsetValue(DEMO_LPADC_BASE, DEMO_LPADC_OFFSET_VALUE_A, DEMO_LPADC_OFFSET_VALUE_B);
#endif /* DEMO_LPADC_DO_OFFSET_CALIBRATION */
#endif /* FSL_FEATURE_LPADC_HAS_OFSTRIM */
    /* Request gain calibration. */
    LPADC_DoAutoCalibration(DEMO_LPADC_BASE);
#endif /* FSL_FEATURE_LPADC_HAS_CTRL_CALOFS */

    /* Set conversion CMD configuration. */
    LPADC_GetDefaultConvCommandConfig(&g_LpadcCommandConfigStruct);
    g_LpadcCommandConfigStruct.channelNumber       = DEMO_LPADC_TEMP_SENS_CHANNEL;
    g_LpadcCommandConfigStruct.sampleChannelMode   = kLPADC_SampleChannelSingleEndSideA;
    g_LpadcCommandConfigStruct.sampleTimeMode      = kLPADC_SampleTimeADCK131;
    g_LpadcCommandConfigStruct.hardwareAverageMode = kLPADC_HardwareAverageCount128;
    g_LpadcCommandConfigStruct.loopCount           = FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE - 1U;
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_MODE) && FSL_FEATURE_LPADC_HAS_CMDL_MODE
    g_LpadcCommandConfigStruct.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif /* FSL_FEATURE_LPADC_HAS_CMDL_MODE */
    LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, DEMO_LPADC_USER_CMDID, &g_LpadcCommandConfigStruct);

    /* Set trigger configuration. */
    LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfigStruct);
    lpadcTriggerConfigStruct.targetCommandId = DEMO_LPADC_USER_CMDID;
    LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &lpadcTriggerConfigStruct); /* Configurate the trigger0. */

    /* Enable the watermark interrupt. */
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2U))
    LPADC_EnableInterrupts(DEMO_LPADC_BASE, kLPADC_FIFO0WatermarkInterruptEnable);
#else
    LPADC_EnableInterrupts(DEMO_LPADC_BASE, kLPADC_FIFOWatermarkInterruptEnable);
#endif /* FSL_FEATURE_LPADC_FIFO_COUNT */
    EnableIRQ(DEMO_LPADC_IRQn);

    /* Eliminate the first two inaccurate results. */
    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* 1U is trigger0 mask. */
    while (false == g_LpadcConversionCompletedFlag)
    {
    }
    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* 1U is trigger0 mask. */
    while (false == g_LpadcConversionCompletedFlag)
    {
    }
}

