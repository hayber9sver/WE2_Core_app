/*
 * out_transport.c
 *
 * See out_transport.h. The probe/timeout periods are counted in idle-loop
 * poll calls rather than wall-clock time (no extra timer dependency), so
 * they are a rough heuristic, not a precise duration.
 */

#include <stdbool.h>
#include "hx_drv_uart.h"
#include "xprintf.h"
#include "out_transport.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define UART1_ID            (DW_UART_1_ID)
#define PROBE_MARKER_BYTE   (0x16) /* ASCII SYN: unlikely to appear from someone just typing */
#define PROBE_PERIOD_POLLS  (200000u)
#define PROBE_TIMEOUT_POLLS (20000u)

static out_transport_e s_transport = OUT_TRANSPORT_USB;
static DEV_UART        *s_uart1 = NULL;
static bool             s_uart1_ready = false;
static bool             s_probe_pending = false;
static uint32_t         s_poll_count = 0;
static uint32_t         s_next_probe_at = PROBE_PERIOD_POLLS;
static uint32_t         s_probe_deadline = 0;

/* cam_task (send_bytes()) and audio_task (read_bytes_nonblock()/send_bytes()
 * and this file's own probe read/write) all touch whatever UART is currently
 * active, and the DW_UART driver has no internal locking of its own. Created
 * here since out_transport_init() runs once, single-threaded, before either
 * task is started.
 *
 * Recursive: callers that stream a large reply in bounded chunks (to avoid
 * one big heap allocation - see event_reply_named_with_payload() in
 * send_result.cpp) hold the lock across the whole multi-chunk send so the
 * other task can't interleave its own message in the middle, while each
 * individual send_bytes()/read_bytes*() call underneath still takes the same
 * lock itself. A plain (non-recursive) mutex would deadlock the owning task
 * on the second, nested take. */
static SemaphoreHandle_t s_uart_mutex = NULL;

void out_transport_lock(void)
{
    if (s_uart_mutex != NULL) {
        xSemaphoreTakeRecursive(s_uart_mutex, portMAX_DELAY);
    }
}

void out_transport_unlock(void)
{
    if (s_uart_mutex != NULL) {
        xSemaphoreGiveRecursive(s_uart_mutex);
    }
}

void out_transport_init(void)
{
    s_uart_mutex = xSemaphoreCreateRecursiveMutex();

    s_uart1 = hx_drv_uart_get_dev((USE_DW_UART_E)UART1_ID);
    if (s_uart1 != NULL) {
        s_uart1->uart_open(UART_BAUDRATE_921600);
        s_uart1_ready = true;
    }
    s_transport = OUT_TRANSPORT_USB;
}

void out_transport_poll(void)
{
    if (s_transport == OUT_TRANSPORT_UART1 || !s_uart1_ready) {
        return;
    }

    s_poll_count++;

    out_transport_lock();

    if (s_probe_pending) {
        char c;
        if (s_uart1->uart_read_nonblock(&c, 1) && (unsigned char)c == PROBE_MARKER_BYTE) {
            s_transport = OUT_TRANSPORT_UART1;
            s_probe_pending = false;
            out_transport_unlock();
            xprintf("out_transport: UART1 echo detected, switching output to UART1\r\n");
            return;
        }
        if (s_poll_count >= s_probe_deadline) {
            s_probe_pending = false; /* no echo within the window, try again later */
        }
        out_transport_unlock();
        return;
    }

    if (s_poll_count >= s_next_probe_at) {
        char b = (char)PROBE_MARKER_BYTE;
        s_uart1->uart_write(&b, 1);
        s_probe_pending = true;
        s_probe_deadline = s_poll_count + PROBE_TIMEOUT_POLLS;
        s_next_probe_at = s_poll_count + PROBE_PERIOD_POLLS;
    }

    out_transport_unlock();
}

out_transport_e out_transport_get(void)
{
    return s_transport;
}

int out_transport_uart_id(void)
{
    return (s_transport == OUT_TRANSPORT_UART1) ? UART1_ID : DW_UART_0_ID;
}
