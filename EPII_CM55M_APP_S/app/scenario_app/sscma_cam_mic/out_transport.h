/*
 * out_transport.h
 *
 * The AT command channel and JSON/base64 result stream both go over UART0,
 * routed through the board's onboard USB-serial bridge (this is the port
 * himax_vision.py / minicom / a direct USB serial connection normally talks
 * to).
 *
 * 2026-07-24: this used to also support switching to UART1 (Grove/pin-header
 * link) for an ESP32C3 bridge board wired there via an active echo-probe
 * handshake - removed along with i2c_cmd.c's I2C command-ingress path once
 * that ESP32C3 bridge project was abandoned in favor of talking to WE2
 * directly over its own USB. UART0's interrupt/DMA-fed RX (below) stays -
 * that solves a real hardware FIFO-overflow problem unrelated to which
 * transport is active.
 */

#ifndef APP_SCENARIO_SSCMA_CAM_MIC_OUT_TRANSPORT_H_
#define APP_SCENARIO_SSCMA_CAM_MIC_OUT_TRANSPORT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UART0 is already opened by board_init()/console_setup(); this just looks
 * up the handle and arms its interrupt/DMA RX chain. */
void out_transport_init(void);

/* 2026-07-11: interrupt/DMA-driven RX, replacing a plain poll of
 * uart_read_nonblock(). WE2's DW_UART RX is a bare 16-byte hardware FIFO -
 * at 921600 baud that fills in ~173us, faster than any FreeRTOS task
 * polling loop (1ms tick granularity) can reliably drain, so bursts longer
 * than 16 bytes would routinely arrive truncated. uart_read_udma() is armed
 * for exactly 1 byte at a time; its completion ISR fires (and immediately
 * re-arms the next 1-byte read) the instant each byte lands, feeding a
 * lock-free single-producer(ISR)/single-consumer(task) ring buffer this
 * pops from - read_bytes_nonblock() (send_result.cpp) uses this instead of
 * touching the DW_UART driver directly. Returns false if nothing is
 * buffered. */
bool out_transport_rx_pop(uint8_t *out_byte);

/* cam_task and audio_task both touch the UART0 peripheral (send_bytes() from
 * send_result.cpp) with no locking in the underlying driver. Every caller
 * must wrap its driver access in this lock/unlock pair.
 *
 * Recursive: callers that stream a large reply in bounded chunks (to avoid
 * one big heap allocation - see event_reply_named_with_payload() in
 * send_result.cpp) hold the lock across the whole multi-chunk send so the
 * other task can't interleave its own message in the middle, while each
 * individual send_bytes() call underneath still takes the same lock itself.
 * A plain (non-recursive) mutex would deadlock the owning task on the
 * second, nested take. */
void out_transport_lock(void);
void out_transport_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_SSCMA_CAM_MIC_OUT_TRANSPORT_H_ */
