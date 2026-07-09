/*
 * sscma_cam_mic.c
 *
 * board_init()/app_main()-style entry point (see sscma_cam_mic_app(),
 * called from app/main.c the same way every other scenario_app's is).
 *
 * Camera and audio run as two independent FreeRTOS tasks (cam_task /
 * audio_task) - per explicit direction, they don't need to start in lockstep,
 * they just both need to be able to run concurrently. cam_task's body is
 * the same hxevent + XDMA-frame-ready pattern allon_sensor_tflm uses
 * (hx_eventloop_start() blocks forever dispatching camera events);
 * audio_task polls incoming AT commands, drains ready PCM chunks, and
 * drives the UART1 echo probe. FreeRTOS's preemptive scheduler (SysTick-
 * driven) is what actually lets audio_task run while cam_task is parked
 * in its blocking event loop - this repo doesn't have a prior example of
 * hxevent running under FreeRTOS (the one FreeRTOS camera app in this SDK,
 * allon_sensor_tflm_freertos, drops hxevent for a hand-rolled task/queue
 * design instead), so this combination is unproven here even though it
 * should work under standard preemptive scheduling.
 */

#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "powermode_export.h"

#include "FreeRTOS.h"
#include "task.h"

#ifdef TRUSTZONE_SEC
#ifdef FREERTOS
#else
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
#endif
#endif

#include "WE2_device.h"
#include "cvapp.h"
#include "board.h"
#include "xprintf.h"
#include "sscma_cam_mic.h"
#include "WE2_core.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#include "spi_eeprom_comm.h"
#ifdef IP_sensorctrl
#include "hx_drv_sensorctrl.h"
#endif
#ifdef IP_xdma
#include "hx_drv_xdma.h"
#include "sensor_dp_lib.h"
#endif
#ifdef IP_cdm
#include "hx_drv_cdm.h"
#endif
#ifdef IP_gpio
#include "hx_drv_gpio.h"
#endif
#include "hx_drv_pmu_export.h"
#include "hx_drv_pmu.h"
#include "powermode.h"
#include "BITOPS.h"

#include "cisdp_sensor.h"
#include "event_handler.h"
#include "common_config.h"

#include "pdm_audio.h"
#include "at_cmd.h"
#include "out_transport.h"

#define CAM_TASK_PRIORITY    (configMAX_PRIORITIES - 2)
#define AUDIO_TASK_PRIORITY  (configMAX_PRIORITIES - 1)
#define CAM_TASK_STACK_WORDS   (4096)
#define AUDIO_TASK_STACK_WORDS (2048)

static uint8_t  g_xdma_abnormal, g_md_detect, g_cdm_fifoerror, g_wdt1_timeout, g_wdt2_timeout, g_wdt3_timeout;
static uint8_t  g_hxautoi2c_error, g_inp1bitparer_abnormal;
static uint32_t g_dp_event;
static uint8_t  g_frame_ready;
static uint32_t g_cur_jpegenc_frame;

static cam_stream_mode_e g_cam_mode = CAM_STREAM_IDLE;
static volatile int g_audio_streaming = 0;
static int      g_result_only = 0;
static uint8_t  g_tscore = 50;

/* -1 = no resolution change pending; otherwise an APP_DP_INP_SUBSAMPLE_E
 * value cam_task should switch to. Set by audio_task (via AT+SENSOR),
 * consumed by cam_task. */
static volatile int g_pending_subs = -1;
static APP_DP_INP_SUBSAMPLE_E g_cam_subs = APP_DP_RES_RGB640x480_INP_SUBSAMPLE_1X;

void app_set_cam_stream_mode(cam_stream_mode_e mode) { g_cam_mode = mode; }
cam_stream_mode_e app_get_cam_stream_mode(void) { return g_cam_mode; }
void app_set_audio_streaming(int active) { g_audio_streaming = active; }
int app_get_audio_streaming(void) { return g_audio_streaming; }
void app_set_result_only(int result_only) { g_result_only = result_only; }
int app_get_result_only(void) { return g_result_only; }
void app_set_tscore(uint8_t score) { g_tscore = score; }
uint8_t app_get_tscore(void) { return g_tscore; }

void app_request_cam_resolution(int subs)
{
    g_pending_subs = subs;
    /* Unblocks cam_task's event_handler_start(), the only way it can ever
     * notice g_pending_subs changed. */
    event_handler_stop();
}

static void dp_var_int(void)
{
    g_xdma_abnormal = 0;
    g_md_detect = 0;
    g_cdm_fifoerror = 0;
    g_wdt1_timeout = 0;
    g_wdt2_timeout = 0;
    g_wdt3_timeout = 0;
    g_inp1bitparer_abnormal = 0;
    g_dp_event = 0;
    g_frame_ready = 0;
    g_cur_jpegenc_frame = 0;
    g_hxautoi2c_error = 0;
}

static void dp_app_cv_eventhdl_cb(EVT_INDEX_E event)
{
    uint16_t err;

    g_dp_event = event;

    switch (event)
    {
    case EVT_INDEX_1BITPARSER_ERR:
        hx_drv_inp1bitparser_get_errstatus(&err);
        hx_drv_inp1bitparser_clear_int();
        hx_drv_inp1bitparser_set_enable(0);
        g_inp1bitparer_abnormal = 1;
        break;
    case EVT_INDEX_EDM_WDT1_TIMEOUT:
        g_wdt1_timeout = 1;
        break;
    case EVT_INDEX_EDM_WDT2_TIMEOUT:
        g_wdt2_timeout = 1;
        break;
    case EVT_INDEX_EDM_WDT3_TIMEOUT:
        g_wdt3_timeout = 1;
        break;
    case EVT_INDEX_CDM_FIFO_ERR:
        g_cdm_fifoerror = 1;
        break;
    case EVT_INDEX_XDMA_WDMA1_ABNORMAL:
    case EVT_INDEX_XDMA_WDMA2_ABNORMAL:
    case EVT_INDEX_XDMA_WDMA3_ABNORMAL:
    case EVT_INDEX_XDMA_RDMA_ABNORMAL:
        g_xdma_abnormal = 1;
        break;
    case EVT_INDEX_CDM_MOTION:
        g_md_detect = 1;
        break;
    case EVT_INDEX_XDMA_FRAME_READY:
        g_cur_jpegenc_frame++;
        g_frame_ready = 1;
        break;
    case EVT_INDEX_HXAUTOI2C_ERR:
        g_hxautoi2c_error = 1;
        break;
    default:
        break;
    }

    if (g_frame_ready == 1)
    {
        g_frame_ready = 0;
        cam_handle_frame(); /* no-op unless AT+INVOKE/AT+SAMPLE started a stream */
        sensordplib_retrigger_capture();
    }

    if (g_md_detect == 1)
    {
        g_md_detect = 0;
    }

    if (g_inp1bitparer_abnormal == 1 || g_wdt1_timeout == 1 || g_wdt2_timeout == 1 || g_wdt3_timeout == 1
            || g_cdm_fifoerror == 1 || g_xdma_abnormal == 1 || g_hxautoi2c_error == 1)
    {
        cisdp_sensor_stop();
    }
}

/* cam_task body: event_handler_start() blocks dispatching camera events
 * until event_handler_stop() is called. Normally this app never calls that,
 * but app_request_cam_resolution() (AT+SENSOR) does, specifically so this
 * loop can wake up, tear down and reinitialize the datapath at the newly
 * requested resolution, and go back to dispatching events - the same
 * stop/reinit/restart sequence edge_impulse_firmware's ei_camera_we2.cpp
 * uses when it restarts streaming (the only other precedent in this repo
 * for calling cisdp_dp_init()/cisdp_sensor_init() more than once). */
static void cam_task(void *pvParameters)
{
    (void)pvParameters;

    if (cisdp_sensor_init() < 0)
    {
        xprintf("\r\nCIS Init fail\r\n");
        APP_BLOCK_FUNC();
    }

    dp_var_int();

    if (cisdp_dp_init(true, SENSORDPLIB_PATH_INT_INP_HW5X5_JPEG, dp_app_cv_eventhdl_cb, 0,
                       g_cam_subs) < 0)
    {
        xprintf("\r\nDATAPATH Init fail\r\n");
        APP_BLOCK_FUNC();
    }

    event_handler_init();
    cisdp_sensor_start();

    for (;;)
    {
        event_handler_start(); /* blocks dispatching camera events until a resolution change is requested */

        int pending = g_pending_subs;
        if (pending < 0)
        {
            break; /* event_handler_stop() called for some other reason - not expected in this app */
        }
        g_pending_subs = -1;
        g_cam_subs = (APP_DP_INP_SUBSAMPLE_E)pending;

        cisdp_sensor_stop();

        dp_var_int();

        if (cisdp_sensor_init() < 0)
        {
            xprintf("\r\nCIS re-init fail\r\n");
            APP_BLOCK_FUNC();
        }

        if (cisdp_dp_init(true, SENSORDPLIB_PATH_INT_INP_HW5X5_JPEG, dp_app_cv_eventhdl_cb, 0,
                           g_cam_subs) < 0)
        {
            xprintf("\r\nDATAPATH re-init fail\r\n");
            APP_BLOCK_FUNC();
        }

        event_handler_init();
        cisdp_sensor_start();
    }

    vTaskDelete(NULL);
}

/* audio_task body: AT-command parsing, audio-chunk draining and the UART1
 * echo probe all live here, independent of whatever cam_task is doing. */
static void audio_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        at_cmd_poll();
        out_transport_poll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

int sscma_cam_mic_app(void)
{
    uint32_t wakeup_event = 0, wakeup_event1 = 0;

    hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
    hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);
    xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x\n", wakeup_event, wakeup_event1);

    /* Required before reading a flash-loaded (FLASH_XIP_MODEL=1) model:
     * without this, tflite::GetModel(MODEL_FLASH_ADDR) reads back whatever
     * happens to be mapped at that CPU address without the QSPI XIP window
     * actually enabled - NOT real flash content, and NOT responsive to
     * anything actually burned there (confirmed on hardware: three
     * different xmodem burns, including a known-good reference model,
     * all read back byte-for-byte identical garbage at 0x3AB7B000 without
     * this call). Every other flash-model app in this repo
     * (tflm_yolov8_od.c, tflm_fd_fm.c) calls this before its own model
     * init - allon_sensor_tflm, which this app's cv_init() scaffolding was
     * copied from, never exercises its own FLASH_XIP_MODEL=1 path in
     * practice, so this call was never carried over. */
    hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q);
    hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, true, FLASH_QUAD, true);

    /* Not fatal to the rest of the app (matches pdm_audio_setup() below):
     * a bad/missing flash-loaded model would otherwise take down UART/
     * audio/camera-JPEG-streaming entirely, when only AI inference itself
     * (AT+INVOKE) actually depends on it - see run_yolo_detect()'s
     * int_ptr==nullptr guard. */
    if (cv_init(true, true) < 0) {
        xprintf("cv init fail (model not loaded - inference will be unavailable)\n");
    }

    if (pdm_audio_setup() != 0) {
        xprintf("pdm_audio setup fail\n");
    }

    out_transport_init();
    at_cmd_init();

    if (xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK_WORDS, NULL, CAM_TASK_PRIORITY, NULL) != pdPASS)
    {
        xprintf("cam_task creation failed\n");
        APP_BLOCK_FUNC();
    }

    if (xTaskCreate(audio_task, "audio_task", AUDIO_TASK_STACK_WORDS, NULL, AUDIO_TASK_PRIORITY, NULL) != pdPASS)
    {
        xprintf("audio_task creation failed\n");
        APP_BLOCK_FUNC();
    }

    vTaskStartScheduler();

    for (;;); /* never reached */
    return 0;
}
