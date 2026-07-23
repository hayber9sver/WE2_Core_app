/*
 * out_transport.c
 *
 * See out_transport.h.
 */

#include <stdbool.h>
#include <string.h>
#include "hx_drv_uart.h"
#include "xprintf.h"
#include "out_transport.h"
#include "WE2_device.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define UART0_ID (DW_UART_0_ID)

static DEV_UART *s_uart0 = NULL;

/* 2026-07-11: interrupt/DMA-driven RX - see out_transport.h's comment on
 * out_transport_rx_pop() for why a plain poll of uart_read_nonblock()
 * wasn't enough. */
#define RX_RING_SIZE (256u) /* power of 2 - cheap index masking */
typedef struct {
    volatile uint8_t  buf[RX_RING_SIZE];
    volatile uint16_t head; /* next write index (ISR-owned) */
    volatile uint16_t tail; /* next read index (task-owned) */
} rx_ring_t;

static rx_ring_t s_rx_ring;

/* DMA target for each in-flight 1-byte read. 32-byte aligned and in its own
 * cache line (Cortex-M55 D-cache line size) - required by
 * hx_drv_pdm_dma_lli_transfer()'s own buffer alignment rule elsewhere in
 * this app (pdm_audio.c) and, more importantly here, to keep
 * SCB_InvalidateDCache_by_Addr() below from invalidating a cache line that
 * also happens to hold unrelated CPU-side data - that would silently
 * discard real writes to whatever else shared the line. */
__attribute__((aligned(32))) static uint8_t s_dma_scratch0[32];

static void uart0_rx_dma_cb(uint32_t status);

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
    rx_ring_push(&s_rx_ring, s_dma_scratch0[0]);
    if (s_uart0 != NULL) {
        s_uart0->uart_read_udma(s_dma_scratch0, 1, (void *)uart0_rx_dma_cb);
    }
}

bool out_transport_rx_pop(uint8_t *out_byte)
{
    return rx_ring_pop(&s_rx_ring, out_byte);
}

/* Created here since out_transport_init() runs once, single-threaded, before
 * either task is started. */
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

    /* UART0 is already opened once by board_init()/console_setup() (see
     * console_io.c) - just look it up, don't uart_open() it again. A
     * redundant reopen briefly resets the peripheral and, worse for the DMA
     * chain below, could plausibly cancel an in-flight uart_read_udma() the
     * same way. */
    s_uart0 = hx_drv_uart_get_dev((USE_DW_UART_E)UART0_ID);

    memset((void *)&s_rx_ring, 0, sizeof(s_rx_ring));
    if (s_uart0 != NULL) {
        s_uart0->uart_read_udma(s_dma_scratch0, 1, (void *)uart0_rx_dma_cb);
    }
}
