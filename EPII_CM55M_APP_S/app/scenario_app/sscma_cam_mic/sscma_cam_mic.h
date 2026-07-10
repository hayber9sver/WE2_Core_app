/*
 * sscma_cam_mic.h
 *
 * Camera (OV5647) + DMIC (PDM) scenario app that speaks a subset of Seeed's
 * AT protocol (see /home/orangepi/SSCMA-Micro/docs/protocol/
 * at-protocol-en_US.md) over UART0, audited/aligned to that spec 2026-07-10
 * - WiFi/MQTT/TRIGGER/ACTION/RC/DFTTPT/KV-store/SSL are all out of scope,
 * this device has no network transport and no persistent KV store:
 *   - AT+INVOKE=<n,differed,result_only>  camera + YOLO gesture inference loop
 *   - AT+SAMPLE=<n>                       raw camera stream, no inference
 *   - AT+ASAMPLE=<n>                      raw DMIC/PDM stream (binary frames, app-specific)
 *   - AT+BREAK                            stop whichever stream is active
 *   - AT+ASR=<rate>/AT+ASR?               set/query the PDM sample rate
 *   - AT+TSCORE=<0..100>/AT+TSCORE?       detection score threshold
 *   - AT+TIOU=<0..100>/AT+TIOU?           NMS IoU threshold
 *   - AT+SENSOR=<id,en,opt>/AT+SENSOR?    set/query camera resolution
 *   - AT+SENSORS?                         list available camera resolutions
 *   - AT+STAT?                            boot count / ready state
 *   - AT+ID?/AT+NAME?/AT+VER?/AT+INFO?    device identity
 *   - AT+RST                              reboot
 *   - AT+HELP?                            command list
 * See at_cmd.h for the parser and cvapp.h/pdm_audio.h for the two data
 * sources.
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_
#define APP_SCENARIO_SSCMA_CAM_MIC_

#include <stdint.h>

#define APP_BLOCK_FUNC() do{ \
	__asm volatile("b    .");\
	}while(0)

typedef enum
{
	APP_STATE_ALLON,
}APP_STATE_E;

/* Camera and audio run as independent FreeRTOS tasks (see sscma_cam_mic.c)
 * and have independent state: AT+INVOKE/AT+SAMPLE/AT+ASAMPLE can all be
 * live at once, AT+BREAK stops both. */
typedef enum {
    CAM_STREAM_IDLE = 0,
    CAM_STREAM_INVOKE,
    CAM_STREAM_SAMPLE,
} cam_stream_mode_e;

#ifdef __cplusplus
extern "C" {
#endif

int sscma_cam_mic_app(void);

void app_set_cam_stream_mode(cam_stream_mode_e mode);
cam_stream_mode_e app_get_cam_stream_mode(void);

void app_set_audio_streaming(int active);
int app_get_audio_streaming(void);

void app_set_result_only(int result_only);
int app_get_result_only(void);

/* 0..100, gates whether the "person" class is reported at all */
void app_set_tscore(uint8_t score_0_100);
uint8_t app_get_tscore(void);

/* 0..100, NMS IoU threshold (AT+TIOU) - see cvapp.cpp's run_nms(). */
void app_set_tiou(uint8_t iou_0_100);
uint8_t app_get_tiou(void);

/* Boot count (AT+STAT?) - incremented once at startup via the AON
 * "appused1" scratch register (see sscma_cam_mic_app()). KNOWN LIMITATION,
 * hardware-confirmed: does not actually survive AT+RST (NVIC_SystemReset()
 * clears appused1 too, not just a real power cycle as originally assumed) -
 * see sscma_cam_mic_app()'s comment. Reads back 1 after every AT+RST rather
 * than incrementing; left as-is rather than adding flash-backed persistence
 * (and its per-boot wear cost) per explicit direction. */
uint32_t app_get_boot_count(void);

/* Requests cam_task switch the camera datapath to a different resolution at
 * runtime (`subs` is an APP_DP_INP_SUBSAMPLE_E value from cisdp_cfg.h - kept
 * as a plain int here so this header doesn't need to pull in the CIS driver
 * headers). Asynchronous: unblocks cam_task's event_handler_start() so it
 * can perform the stop/reinit/restart sequence itself; the new resolution
 * takes effect within the next couple of frames, not synchronously. */
void app_request_cam_resolution(int subs);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_ */
