/*
 * pdm_audio.h
 *
 * Lightweight PDM/DMIC capture module for sscma_cam_mic.
 * Adapted from kws_pdm_record's PDM DMA ping-pong pattern, minus the
 * KWS/MFCC inference path: this module only hands back raw PCM chunks
 * for the caller to base64-encode and stream out over UART.
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_PDM_AUDIO_H_
#define APP_SCENARIO_SSCMA_CAM_MIC_PDM_AUDIO_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Supported PCM sample rates (same set the PDM IP supports). */
#define PDM_AUDIO_DEFAULT_SAMPLE_RATE_HZ (16000)

/* One-time pin mux + driver init at the configured sample rate. */
int pdm_audio_setup(void);

/* De-init the PDM peripheral (stops sampling). Safe to call when stopped. */
void pdm_audio_teardown(void);

/* Change the sample rate (8000/16000/32000/48000). If currently running,
 * the module is stopped, reconfigured and restarted automatically.
 * Returns 0 on success, -1 on unsupported rate. */
int pdm_audio_set_rate(uint32_t sample_rate_hz);

uint32_t pdm_audio_get_rate(void);

/* Start/stop the DMA ping-pong capture loop. */
int pdm_audio_start(void);
void pdm_audio_stop(void);
bool pdm_audio_is_running(void);

/* Poll for a completed chunk of audio.
 * Returns a pointer to the chunk (valid until the next call) and its
 * length in bytes via *out_len_bytes, or NULL if nothing new is ready. */
const int16_t *pdm_audio_poll_chunk(uint32_t *out_len_bytes);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_PDM_AUDIO_H_ */
