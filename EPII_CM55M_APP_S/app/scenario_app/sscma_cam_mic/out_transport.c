/*
 * out_transport.c
 *
 * See out_transport.h. The probe/timeout periods are counted in idle-loop
 * poll calls rather than wall-clock time (no extra timer dependency), so
 * they are a rough heuristic, not a precise duration.
 */

#include <stdbool.h>
#include <string.h>
#include "hx_drv_uart.h"
#include "hx_drv_scu.h"
#include "xprintf.h"
#include "out_transport.h"
#include "WE2_device.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define UART0_ID             (DW_UART_0_ID)
#define UART1_ID             (DW_UART_1_ID)
#define PROBE_MARKER_BYTE    (0x16) /* ASCII SYN: unlikely to appear from someone just typing */
/* 2026-07-10: was 200000/20000 - at the ~5ms/poll cadence out_transport_poll()
 * actually runs at (audio_task's vTaskDelay(5), sscma_cam_mic.c), that's a
 * ~1000s (16.7 min) period with only a ~100s echo window per attempt - found
 * while bringing up an ESP32C3 bridge on UART1: nobody, tester or real user,
 * should have to wait 17 minutes for a freshly-plugged-in peer to be
 * detected. Shortened to a ~5s period / ~2s window - still cheap (a single
 * byte write plus a couple of register reads every 5s is noise), and still
 * "rough heuristic" per this file's own header comment, just a much more
 * usable one. */
#define PROBE_PERIOD_POLLS   (1000u)
#define PROBE_TIMEOUT_POLLS  (400u)

static out_transport_e s_transport      = OUT_TRANSPORT_USB;
static DEV_UART        *s_uart0         = NULL;
static DEV_UART        *s_uart1         = NULL;
static bool             s_uart1_ready   = false;
static bool             s_probe_pending = false;
static uint32_t         s_poll_count    = 0;
static uint32_t         s_next_probe_at = PROBE_PERIOD_POLLS;
static uint32_t         s_probe_deadline = 0;

/* 2026-07-11: interrupt/DMA-driven RX, replacing what used to be a plain
 * poll of uart_read_nonblock() from audio_task's own loop (at_cmd_poll() in
 * at_cmd.cpp, previously via read_bytes_nonblock() in send_result.cpp).
 *
 * Root cause this replaces: WE2's DW_UART RX is a bare 16-byte hardware
 * FIFO. At 921600 baud that fills in ~173us. audio_task's poll loop - even
 * after shortening its vTaskDelay from 5ms to 1ms - simply cannot guarantee
 * catching every burst inside that window, since 1ms is still ~6x longer
 * than the fill time; anything arriving faster than the FIFO drains
 * silently overflows and gets dropped by hardware, with no software-visible
 * error. Confirmed repeatedly on hardware: an ESP32C3 bridge sending
 * AT+<tag>@<body>\r\n bursts >16 bytes (any tagged command with a body
 * longer than a few characters, e.g. AT+SENSOR=1,1,0) would routinely
 * arrive truncated exactly at that boundary.
 *
 * A hardware FIFO can only be guaranteed never to overflow by something
 * that drains it with sub-173us latency, which no FreeRTOS task polling
 * loop can promise (tick granularity alone is 1ms). Interrupt-driven DMA
 * reads can: uart_read_udma() below is armed for exactly 1 byte at a time,
 * so the completion ISR fires (and immediately re-arms the next 1-byte
 * read) the instant each byte lands, regardless of what any task is doing.
 * The pattern (uart_read_udma() + a completion callback that re-arms
 * itself) mirrors event_handler/evt_uartcomm.c's own working use of this
 * same driver API - the one other place in this whole SDK that touches
 * uart_read_udma() at all - except that one defers its re-arm to task
 * context via hx_event_activate_ISR() (fine for its fixed-size, low-rate
 * OTA command protocol). This can't afford that extra hop: re-arming only
 * once every ~1ms task cycle would just reintroduce the exact same
 * FIFO-overflow window this is meant to close. uart_read_udma() is
 * documented as non-blocking (register-level DMA descriptor setup, no
 * blocking wait), which is what makes calling it directly from the
 * completion ISR viable here.
 *
 * Two independent single-byte DMA chains run continuously from
 * out_transport_init() onward, one per UART, each feeding its own
 * lock-free single-producer(ISR)/single-consumer(task) ring buffer. Only
 * one of the two ever has genuine traffic at a time (matching
 * out_transport_uart_id()'s own single-active-transport model): out_transport_
 * poll()'s probe-echo detection drains the UART1 ring pre-switch, and
 * out_transport_rx_pop() (used by read_bytes_nonblock() in
 * send_result.cpp) drains whichever ring matches out_transport_uart_id()
 * post-switch. Keeping both chains always-armed (rather than tearing one
 * down and arming the other exactly at the switch instant) avoids needing
 * a UART_CMD_ABORT_RX/re-arm handoff at a precise moment - simpler, and
 * nothing reads the inactive side's ring, so it just sits there unused. */
#define RX_RING_SIZE (256u) /* power of 2 - cheap index masking */
typedef struct {
    volatile uint8_t  buf[RX_RING_SIZE];
    volatile uint16_t head; /* next write index (ISR-owned) */
    volatile uint16_t tail; /* next read index (task-owned) */
} rx_ring_t;

static rx_ring_t s_rx_ring[2]; /* index 0 = UART0, index 1 = UART1 */

/* DMA target for each in-flight 1-byte read. 32-byte aligned and each in
 * its own cache line (Cortex-M55 D-cache line size) - required by
 * hx_drv_pdm_dma_lli_transfer()'s own buffer alignment rule elsewhere in
 * this app (pdm_audio.c) and, more importantly here, to keep
 * SCB_InvalidateDCache_by_Addr() below from invalidating a cache line that
 * also happens to hold unrelated CPU-side data - that would silently
 * discard real writes to whatever else shared the line. */
__attribute__((aligned(32))) static uint8_t s_dma_scratch0[32];
__attribute__((aligned(32))) static uint8_t s_dma_scratch1[32];

static void uart0_rx_dma_cb(uint32_t status);
static void uart1_rx_dma_cb(uint32_t status);

static inline bool rx_ring_push(rx_ring_t *r, uint8_t b)
{
    uint16_t next = (uint16_t)((r->head + 1u) & (RX_RING_SIZE - 1u));
    if (next == r->tail) {
        return false; /* full - drop the byte rather than overwrite unread data */
    }
    r->buf[r->head] = b;
    r->head = next;
    return true;
}

static inline bool rx_ring_pop(rx_ring_t *r, uint8_t *out)
{
    if (r->tail == r->head) {
        return false;
    }
    *out = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) & (RX_RING_SIZE - 1u));
    return true;
}

static void uart0_rx_dma_cb(uint32_t status)
{
    (void)status;
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_dma_scratch0, sizeof(s_dma_scratch0));
    rx_ring_push(&s_rx_ring[0], s_dma_scratch0[0]);
    if (s_uart0 != NULL) {
        s_uart0->uart_read_udma(s_dma_scratch0, 1, (void *)uart0_rx_dma_cb);
    }
}

static void uart1_rx_dma_cb(uint32_t status)
{
    (void)status;
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_dma_scratch1, sizeof(s_dma_scratch1));
    rx_ring_push(&s_rx_ring[1], s_dma_scratch1[0]);
    if (s_uart1 != NULL) {
        s_uart1->uart_read_udma(s_dma_scratch1, 1, (void *)uart1_rx_dma_cb);
    }
}

/* Pops one byte from whichever UART's ring is currently the active
 * transport - read_bytes_nonblock() (send_result.cpp) uses this instead of
 * touching the DW_UART driver directly. */
bool out_transport_rx_pop(uint8_t *out_byte)
{
    rx_ring_t *r = (s_transport == OUT_TRANSPORT_UART1) ? &s_rx_ring[1] : &s_rx_ring[0];
    return rx_ring_pop(r, out_byte);
}

/* cam_task (send_bytes()) and audio_task (send_bytes(), and this file's own
 * probe write) both touch whatever UART is currently active for *writes*,
 * and the DW_UART driver has no internal locking of its own for that.
 * Reads no longer need this lock (the ISR-fed ring buffers are already
 * lock-free single-producer/single-consumer) - still held here only around
 * the write path and the probe/switch state machine. Created here since
 * out_transport_init() runs once, single-threaded, before either task is
 * started.
 *
 * Recursive: callers that stream a large reply in bounded chunks (to avoid
 * one big heap allocation - see event_reply_named_with_payload() in
 * send_result.cpp) hold the lock across the whole multi-chunk send so the
 * other task can't interleave its own message in the middle, while each
 * individual send_bytes() call underneath still takes the same lock itself.
 * A plain (non-recursive) mutex would deadlock the owning task on the
 * second, nested take. */
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

    /* 2026-07-10: found missing while bringing up an ESP32C3 bridge on
     * UART1 - hx_drv_uart_get_dev()/uart_open()/uart_write() below all
     * report success with NO pinmux call at all, because they operate on
     * the DW_UART peripheral itself, not the physical pins. Without
     * explicitly routing the right pins to the UART1 function, the probe
     * byte never actually reaches the Grove/pin-header connector -
     * confirmed on hardware: probe writes returned ret=1 (success) every
     * cycle while an external UART listener wired to the header saw zero
     * bytes, ever.
     *
     * PB7=UART1_TX / PB6=UART1_RX, per the actual board schematic
     * (Grove_Vision_AI_Module_V2_Circuit_Diagram.pdf, net labels
     * "PB7/UART1_TX/3V3" and "PB6/UART1_RX/3V3" on the HX6538 pins) - NOT
     * PB5, which is what event_handler's evt_uartcomm.c (a different
     * subsystem, its own unrelated UART1 users) uses for its TX pin. That
     * looked like a reasonable pattern to copy at first (same UART1
     * peripheral), and it even compiles/"succeeds" - the HX6538's pin
     * crossbar genuinely lets UART1_TX be muxed onto *either* PB5 or PB7
     * (SCU_PB5_PINMUX_E and SCU_PB7_PINMUX_E both have a UART1_TX = 10
     * option), so hx_drv_scu_set_PB5_pinmux(..., UART1_TX, ...) is not
     * itself an error. It's just the wrong *physical* choice for this
     * board: this PCB's Grove/pin-header connector is only copper-traced to
     * PB7, not PB5, so muxing UART1_TX onto PB5 routes the signal to a pin
     * that (on this board) isn't connected to anything external - confirmed
     * by a hardware loopback test producing zero signal with PB5, then
     * cross-checking the schematic. Don't copy evt_uartcomm.c's pin choice
     * again without re-checking which physical pin this board actually
     * brings out. */
    hx_drv_scu_set_PB7_pinmux(SCU_PB7_PINMUX_UART1_TX, 1);
    hx_drv_scu_set_PB6_pinmux(SCU_PB6_PINMUX_UART1_RX, 1);

    /* UART0 is already opened once by board_init()/console_setup() (see
     * console_io.c) - just look it up, don't uart_open() it again. A
     * redundant reopen briefly resets the peripheral (see get_console_uart()'s
     * old comment in send_result.cpp, before this file took over RX) and,
     * worse for the DMA chain below, could plausibly cancel an in-flight
     * uart_read_udma() the same way. */
    s_uart0 = hx_drv_uart_get_dev((USE_DW_UART_E)UART0_ID);

    s_uart1 = hx_drv_uart_get_dev((USE_DW_UART_E)UART1_ID);
    if (s_uart1 != NULL) {
        /* Reverted 2026-07-11 back to 921600 - the real ESP32C3 bridge
         * generates an accurate 921600 clock and needs the full throughput
         * (concurrent camera+audio streaming doesn't fit in 115200's
         * ~11.5KB/s, confirmed by hardware testing). 115200 was only ever a
         * temporary accommodation for Orange Pi's ttyS5, whose 24MHz clock
         * has no clean divisor near 921600. */
        s_uart1->uart_open(UART_BAUDRATE_921600);
        s_uart1_ready = true;
        xprintf("out_transport: UART1 (id=%d) opened OK, probe armed\r\n", UART1_ID);
    } else {
        xprintf("out_transport: UART1 (id=%d) hx_drv_uart_get_dev() returned NULL - "
                "probe disabled, UART1 auto-switch will never happen\r\n", UART1_ID);
    }
    s_transport = OUT_TRANSPORT_USB;

    memset((void *)&s_rx_ring[0], 0, sizeof(s_rx_ring[0]));
    memset((void *)&s_rx_ring[1], 0, sizeof(s_rx_ring[1]));
    if (s_uart0 != NULL) {
        s_uart0->uart_read_udma(s_dma_scratch0, 1, (void *)uart0_rx_dma_cb);
    }
    if (s_uart1 != NULL) {
        s_uart1->uart_read_udma(s_dma_scratch1, 1, (void *)uart1_rx_dma_cb);
    }
}

void out_transport_poll(void)
{
    if (s_transport == OUT_TRANSPORT_UART1 || !s_uart1_ready) {
        return;
    }

    s_poll_count++;

    out_transport_lock();

    if (s_probe_pending) {
        uint8_t c;
        /* Always UART1's own ring here, not out_transport_rx_pop() (which
         * follows s_transport - still USB/UART0 at this point, since
         * detecting whether to switch is exactly what this block does). */
        if (rx_ring_pop(&s_rx_ring[1], &c) && c == PROBE_MARKER_BYTE) {
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
        int32_t written = s_uart1->uart_write(&b, 1);
        xprintf("out_transport: probe write on UART1, ret=%ld\r\n", (long)written);
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
