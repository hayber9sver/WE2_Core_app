/*
 * cvapp.h
 *
 * Camera-side pipeline for sscma_cam_mic: person-detect inference on the
 * Ethos-U55 (copied from allon_sensor_tflm) plus the SSCMA-style JSON/
 * base64 reply, gated by whatever stream mode at_cmd.cpp last set
 * (see sscma_cam_mic.h). Kept as one C-callable entry point per frame so
 * the plain-C frame-ready event handler in sscma_cam_mic.c doesn't need to
 * touch any C++ (std::string/forward_list) itself.
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_CVAPP_
#define APP_SCENARIO_SSCMA_CAM_MIC_CVAPP_

#include "spi_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

int cv_init(bool security_enable, bool privilege_enable);

/* Call once per camera frame-ready event. No-ops when the stream mode is
 * STREAM_MODE_IDLE; otherwise runs inference (STREAM_MODE_CAM_INVOKE) or
 * not (STREAM_MODE_CAM_SAMPLE) and sends the appropriate JSON/base64
 * packet over the active transport. */
void cam_handle_frame(void);

int cv_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_CVAPP_ */
