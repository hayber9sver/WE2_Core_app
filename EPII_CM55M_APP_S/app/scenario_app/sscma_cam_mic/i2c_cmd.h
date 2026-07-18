/*
 * i2c_cmd.h
 *
 * I2C slave 0 (PA2=SCL/PA3=SDA, WE2 Grove I2C connector) command-in channel:
 * ESP32 (I2C master) writes raw AT command text ("AT+INVOKE=-1,0,0\r\n",
 * same bytes camera_web_server.ino's sendTaggedCommand() already builds for
 * the UART path) directly, no custom framing. Received lines are handed to
 * at_cmd_process_line() (at_cmd.h) - the exact same dispatcher UART commands
 * go through - so every reply still goes out over UART via send_bytes(),
 * unchanged. This is a second, additional command *ingress* path, not a
 * replacement for UART; out_transport.c/at_cmd.cpp are untouched.
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_I2C_CMD_H_
#define APP_SCENARIO_SSCMA_CAM_MIC_I2C_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Pin mux (PA2/PA3) + I2C slave 0 driver init + first read arm. Call once,
 * after out_transport_init()/at_cmd_init() (sscma_cam_mic.c). */
void i2c_cmd_init(void);

/* Drains a completed I2C write from the master if one is pending, dispatches
 * it, and re-arms the next read. Call every idle-loop tick, alongside
 * at_cmd_poll()/out_transport_poll(); non-blocking. */
void i2c_cmd_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_I2C_CMD_H_ */
