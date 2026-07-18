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

/* 2026-07-10: tag support (AT+<Tag>@<Body> - at-protocol-en_US.md's
 * "Tagging" section), added specifically so the ESP32C3 camera_web_server
 * bridge's command_handler() (which tags every command it relays, e.g.
 * "AT+HTTPD1a2b3c4d@SENSOR=1,1,2") can correlate replies to requests -
 * before this, every tagged command fell through to "unsupported command"
 * because the whole "<tag>@<name>" string never matched any known command
 * name. Returns "" (never NULL) when the command currently being processed
 * had no tag. Only valid to call while a reply to that command is being
 * built (i.e. from inside a command handler or a send_*_reply() function
 * called from one) - the value is scratch, reset per line. */
const char *at_cmd_current_tag(void);

/* 2026-07-16: shared dispatch entry point for command bytes that arrive
 * from somewhere other than the UART line-accumulation loop in
 * at_cmd_poll() - added for i2c_cmd.c's I2C command channel. `line` must
 * already be a NUL-terminated, mutable buffer with no trailing '\r'/'\n'
 * (same contract at_cmd_poll() gives process_line() internally) - the
 * parser tokenizes it in place. */
void at_cmd_process_line(char *line);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_AT_CMD_H_ */
