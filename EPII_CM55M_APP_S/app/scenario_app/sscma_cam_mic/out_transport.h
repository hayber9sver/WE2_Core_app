/*
 * out_transport.h
 *
 * Picks which physical serial link carries the AT command channel and the
 * JSON/base64 result stream:
 *   - default: UART0, routed through the board's onboard USB-serial bridge
 *     (this is the port himax_vision.py / minicom normally talks to)
 *   - UART1, broken out on the Grove/pin header, once something is
 *     actually plugged in there
 *
 * Detection is an active echo probe (per user direction, not passive
 * listening): while still on the default transport, periodically write a
 * single marker byte out UART1 and see whether it comes back within a
 * short window (a wired loopback / echoing peer on the other end). On a
 * match we switch permanently to UART1 for the rest of the session.
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_OUT_TRANSPORT_H_
#define APP_SCENARIO_SSCMA_CAM_MIC_OUT_TRANSPORT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OUT_TRANSPORT_USB   = 0, /* UART0 via the onboard USB-serial bridge (default) */
    OUT_TRANSPORT_UART1 = 1, /* UART1, external Grove/pin-header link */
} out_transport_e;

/* Opens UART1 (UART0 is already opened by board_init()/console_setup()). */
void out_transport_init(void);

/* Drives the echo-probe state machine. Call from the idle loop. A no-op
 * once already switched to UART1. */
void out_transport_poll(void);

out_transport_e out_transport_get(void);

/* USE_DW_UART_E id of whichever transport is currently active; use this
 * instead of a hardcoded CONSOLE_UART_ID when opening/reading/writing the
 * console UART. */
int out_transport_uart_id(void);

/* 2026-07-11: pops one byte (if available) from the interrupt/DMA-fed ring
 * buffer for whichever UART out_transport_uart_id() currently reports -
 * read_bytes_nonblock() (send_result.cpp) uses this instead of touching the
 * DW_UART driver directly, so incoming bytes get picked up the instant they
 * land rather than whenever a task next happens to poll (see out_transport.c's
 * own comment for why that distinction matters: the RX hardware FIFO is only
 * 16 bytes, and overflows well before any task polling interval could
 * reliably catch it). Returns false if nothing is buffered. */
bool out_transport_rx_pop(uint8_t *out_byte);

/* cam_task and audio_task both end up touching whichever UART peripheral is
 * currently active (send_bytes()/read_bytes*() from send_result.cpp, and the
 * UART1 echo probe here) with no locking in the underlying driver. Every
 * caller must wrap its driver access in this lock/unlock pair. */
void out_transport_lock(void);
void out_transport_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_OUT_TRANSPORT_H_ */
