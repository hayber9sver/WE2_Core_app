/*
 * at_cmd.h
 *
 * Minimal from-scratch AT-command line parser speaking the subset of the
 * SSCMA/himax_vision.py dialect this app needs (AT+INVOKE, AT+SAMPLE,
 * AT+BREAK, AT+SENSOR, AT+TSCORE, AT+INFO?/NAME?/VER?/ID?/MODEL?) plus two
 * app-specific audio commands (AT+ASAMPLE, AT+ASR). Lines are '\r'- or
 * '\n'-terminated, e.g. "AT+INVOKE=-1,0,0\r".
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_AT_CMD_H_
#define APP_SCENARIO_SSCMA_CAM_MIC_AT_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

void at_cmd_init(void);

/* Drains whatever bytes are waiting on the active transport (see
 * out_transport.h) and feeds them to the line parser. Call every idle-loop
 * tick; non-blocking. */
void at_cmd_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_AT_CMD_H_ */
