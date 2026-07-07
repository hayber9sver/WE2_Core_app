/*
 * pdm_audio.c
 *
 * See pdm_audio.h. Config values (oversampling_ratio/cic_stages/cic_delay/
 * dc_removal/bit_range_shift) are copied verbatim from kws_pdm_record and
 * pdm_record, which only ever vary pdm_cfg.sample_rate across the four
 * supported rates and keep the rest fixed.
 */

#include <string.h>
#include "WE2_device.h"
#include "hx_drv_scu.h"
#include "hx_drv_pdm_rx.h"
#include "xprintf.h"
#include "pdm_audio.h"

#define CHUNK_SAMPLES   (4000)                 /* samples per chunk/DMA block (int16, mono) */
#define CHUNK_BYTES     (CHUNK_SAMPLES * 2)    /* 8000 bytes/block, matching the reference
                                                * pdm_record.c (QUARTER_SECOND_MONO_BYTES) and
                                                * staying under the <8192 driver limit */

/* Each hx_drv_pdm_dma_lli_transfer() call covers BLOCKS_PER_XFER contiguous
 * chunks. Within one call the LLI (linked-list) DMA engine chains those blocks
 * gaplessly; the only re-arm seam (where the DMA has stopped and pdm_audio_rx_cb()
 * must kick the next transfer, during which the PDM FIFO can overflow and drop a
 * few samples -> a boundary click) happens once per transfer instead of once per
 * chunk. With the 8000-byte block size (per the reference) plus the last-sample
 * repair in pdm_audio_poll_chunk(), the dominant per-block click is gone; this
 * multi-block ping-pong then keeps any residual re-arm seam rare too. Transfer
 * span = BLOCKS_PER_XFER * CHUNK = 8000 samples (~0.5s at 16k / ~0.25s at 32k).
 * NUM_XFER=2 gives a ping-pong: one region is drained while the other fills.
 * Tunable. */
#define BLOCKS_PER_XFER (2)                    /* chunks per DMA transfer (gapless within) */
#define NUM_XFER        (2)                    /* ping-pong region count */
#define NUM_BUFF        (BLOCKS_PER_XFER * NUM_XFER)  /* total ring depth (4) */

/* hx_drv_pdm_dma_lli_transfer() hard-requires a 32-byte-aligned buffer: pass
 * an unaligned one and it prints a warning and returns an error *without*
 * starting the transfer, so pdm_audio_rx_cb() never fires and every chunk
 * poll silently returns NULL forever. Confirmed by disassembling the
 * prebuilt libdriver.a - there's no source for it in this tree. */
/* Placed in .bss.NoInit (the big 2MB CM55M_S_SRAM region, same as cvapp.cpp's
 * tensor arena) rather than the default .bss - this ring (NUM_BUFF * 8000 bytes)
 * plus the 80KB heap and 64KB stack would otherwise crowd the small 256KB
 * CM55M_S_APP_DATA region. NOLOAD is fine: the DMA fills every buffer before the
 * consumer reads it, so it never relies on zero-init. */
__attribute__((section(".bss.NoInit"), aligned(32)))
static int16_t s_audio_buf[NUM_BUFF][CHUNK_SAMPLES];
/* Repaired copy handed to the caller (see pdm_audio_poll_chunk). Small enough
 * to sit in the default .bss/DATA region; the consumer sends it before the next
 * poll, so a single shared scratch buffer is safe. */
static int16_t s_chunk_out[CHUNK_SAMPLES] __attribute__((aligned(32)));
static PDM_DEV_INFO s_pdm_dev_info;
static uint32_t s_sample_rate_hz = PDM_AUDIO_DEFAULT_SAMPLE_RATE_HZ;
/* Monotonic chunk counters (not ring indices) so there's no full-vs-empty
 * wrap ambiguity: [s_r_count, s_w_count) is the readable backlog, physical
 * buffer for chunk N is N % NUM_BUFF. */
static volatile uint32_t s_w_count = 0;   /* chunks captured & published so far */
static uint32_t s_r_count = 0;            /* chunks handed to the caller so far */
static volatile bool s_running = false;
static bool s_pdm_inited = false;
static bool s_dma_armed = false;          /* has the free-running chain been kicked off? */

static void pdm_audio_rx_cb(uint32_t int_status)
{
    (void)int_status;
    /* The BLOCKS_PER_XFER-chunk region that just finished occupied physical
     * buffers [s_w_count % NUM_BUFF ..]; publish them by advancing the counter,
     * then re-arm the next region (BLOCKS_PER_XFER divides NUM_BUFF, so the base
     * is always region-aligned). This callback keeps re-arming unconditionally
     * regardless of s_running - the DMA ping-pong chain must free-run forever
     * once started (see pdm_audio_stop()/pdm_audio_set_rate() comments). */
    s_w_count += BLOCKS_PER_XFER;
    int32_t base = (int32_t)(s_w_count % NUM_BUFF);
    hx_drv_pdm_dma_lli_transfer((void *)s_audio_buf[base], BLOCKS_PER_XFER, CHUNK_BYTES, 0);
}

/* Only 16k/32k are supported (see pdm_audio_set_rate). */
static PDM_PCM_FREQ_E rate_to_enum(uint32_t rate_hz)
{
    switch (rate_hz) {
    case 16000: return PDM_PCM_FREQ_16K;
    case 32000: return PDM_PCM_FREQ_32K;
    default:    return PDM_PCM_FREQ_16K;
    }
}

static void build_cfg(void)
{
    memset(&s_pdm_dev_info, 0, sizeof(s_pdm_dev_info));
    s_pdm_dev_info.pdm_cfg.reg_addr = HIMAX_PDM_BASE_ADDR;
    s_pdm_dev_info.pdm_cfg.rx_fifo_threshold = 5;
    /* Fixed at 8 for the whole app lifetime. The reference apps note "6: 32KHz,
     * 8: 16KHz" (oversampling should track the rate), but there is NO runtime
     * setter for oversampling in this driver - it's only applied inside
     * hx_drv_pdm_init(), which we deliberately call exactly once (a second
     * init/deinit permanently breaks the PDM DMA - the bug #6 finding). Runtime
     * rate changes therefore go through hx_drv_pdm_set_pcm_freq() only, which
     * doesn't touch oversampling. 8 is correct for the 16k boot default; whether
     * set_pcm_freq internally compensates the clocking so 32k is also clean at
     * oversampling=8 has to be judged by ear (16k already shows the boundary
     * click, i.e. the click is independent of this). */
    s_pdm_dev_info.pdm_cfg.oversampling_ratio = 8;
    s_pdm_dev_info.pdm_cfg.cic_stages = 0;
    s_pdm_dev_info.pdm_cfg.cic_delay = 0;
    s_pdm_dev_info.pdm_cfg.dc_removal = 6;
    s_pdm_dev_info.pdm_cfg.bit_range_shift = 5;

    s_pdm_dev_info.pdm_cfg.data_in_0_en = PDM_DATA_IN_ENABLE;
    s_pdm_dev_info.pdm_cfg.data_in_1_en = PDM_DATA_IN_DISABLE;
    s_pdm_dev_info.pdm_cfg.data_in_2_en = PDM_DATA_IN_DISABLE;
    s_pdm_dev_info.pdm_cfg.data_in_3_en = PDM_DATA_IN_DISABLE;
    s_pdm_dev_info.pdm_cfg.capture_channel = PDM_CAPTURE_CHANNEL_LEFT_ONLY;

    s_pdm_dev_info.pdm_cfg.dma_ch = PDM_USE_DMA2_CH_0;
    s_pdm_dev_info.pdm_cfg.pdm_clk_src = PDM_CLKSRC_LSCREF;
    s_pdm_dev_info.pdm_cfg.sample_rate = rate_to_enum(s_sample_rate_hz);
    s_pdm_dev_info.pdm_rx_cb = pdm_audio_rx_cb;
    s_pdm_dev_info.pdm_err_cb = NULL;
}

int pdm_audio_setup(void)
{
    /* PDM CLK (PB9) / PDM DATA0 (PB10), same pins as kws_pdm_record / pdm_record */
    hx_drv_scu_set_PB9_pinmux(SCU_PB9_PINMUX_PDM_CLK_13, 1);
    hx_drv_scu_set_PB10_pinmux(SCU_PB10_PINMUX_PDM_DATA0_13, 1);

    build_cfg();
    if (hx_drv_pdm_init(&s_pdm_dev_info) != PDM_NO_ERROR) {
        xprintf("pdm_audio: init fail\r\n");
        return -1;
    }
    s_pdm_inited = true;
    return 0;
}

void pdm_audio_teardown(void)
{
    pdm_audio_stop();
    if (s_pdm_inited) {
        hx_drv_pdm_deinit();
        s_pdm_inited = false;
    }
}

/* Changing rate used to go through a full hx_drv_pdm_deinit()+hx_drv_pdm_init()
 * cycle. Confirmed on hardware across three separate attempts that ANY
 * explicit stop/deinit intervention here (hx_drv_pdm_stop_transfer() alone,
 * hx_drv_pdm_stop_transfer() only right before deinit, or a full
 * deinit+init on every single start/stop) breaks the *next* restart -
 * even a same-rate one - after the very first cycle. Only ever calling
 * hx_drv_pdm_init()/hx_drv_pdm_deinit() ONCE for the entire app lifetime
 * (at pdm_audio_setup()) and otherwise leaving the peripheral alone is the
 * one pattern that reliably supports repeated AT+ASAMPLE/AT+BREAK start/stop
 * cycling (pdm_audio_rx_cb()'s DMA ping-pong chain just keeps running
 * regardless, and pdm_audio_start()/pdm_audio_stop() gate exposure to it
 * with a plain software flag - see their comments). So rate changes now use
 * hx_drv_pdm_set_pcm_freq() instead, a dedicated driver call for exactly
 * this that doesn't touch init/deinit at all. */
int pdm_audio_set_rate(uint32_t sample_rate_hz)
{
    /* Only 16k/32k: those are the two rates with a known-good oversampling
     * value (16k=8, 32k=6 per the reference) and, since oversampling can't be
     * changed at runtime (see build_cfg), the two we can actually stand behind.
     * 8k/48k are rejected. */
    if (sample_rate_hz != 16000 && sample_rate_hz != 32000) {
        return -1;
    }
    if (sample_rate_hz == s_sample_rate_hz) {
        return 0;
    }

    if (s_pdm_inited) {
        if (hx_drv_pdm_set_pcm_freq(rate_to_enum(sample_rate_hz)) != PDM_NO_ERROR) {
            xprintf("pdm_audio: set_pcm_freq at %u Hz fail\r\n", (unsigned)sample_rate_hz);
            return -1;
        }
    }
    s_sample_rate_hz = sample_rate_hz;
    build_cfg(); /* keeps s_pdm_dev_info.pdm_cfg.sample_rate in sync for consistency */
    return 0;
}

uint32_t pdm_audio_get_rate(void)
{
    return s_sample_rate_hz;
}

int pdm_audio_start(void)
{
    if (!s_pdm_inited) {
        return -1;
    }
    if (s_running) {
        return 0;
    }
    if (!s_dma_armed) {
        /* First ever start: kick off the free-running ping-pong chain. From
         * here on pdm_audio_rx_cb() re-arms it forever, so we never touch the
         * DMA again (arming a second transfer while one is in flight is exactly
         * what we're avoiding). */
        s_w_count = 0;
        s_r_count = 0;
        hx_drv_pdm_dma_lli_transfer((void *)s_audio_buf[0], BLOCKS_PER_XFER, CHUNK_BYTES, 0);
        s_dma_armed = true;
    } else {
        /* Restart: the chain has been running (and overwriting the ring) the
         * whole time we were stopped, so drop the stale backlog and stream from
         * the current write position - fresh samples only. */
        s_r_count = s_w_count;
    }
    s_running = true;
    return 0;
}

void pdm_audio_stop(void)
{
    /* Deliberately just a software gate on pdm_audio_poll_chunk() - see
     * pdm_audio_set_rate()'s comment for why this needs to stay this way
     * (the DMA ping-pong chain keeps running in hardware regardless, and
     * repeated start/stop cycling relies on that). */
    s_running = false;
}

bool pdm_audio_is_running(void)
{
    return s_running;
}

const int16_t *pdm_audio_poll_chunk(uint32_t *out_len_bytes)
{
    if (!s_running) {
        return NULL;
    }
    /* Chunks in [s_r_count, s_w_count) are fully captured. The consumer polls
     * every ~5ms and drains one per poll, so it stays far ahead of the DMA
     * (~8 chunks/s at 32k with 4000-sample chunks) and never lags a full ring
     * behind - no overrun/full-vs-empty ambiguity in practice. */
    if (s_r_count == s_w_count) {
        return NULL; /* nothing new yet */
    }
    const int16_t *src = s_audio_buf[s_r_count % NUM_BUFF];
    SCB_InvalidateDCache_by_Addr((uint32_t *)src, CHUNK_BYTES);

    /* Copy out to a scratch buffer rather than handing back (and modifying) the
     * live DMA buffer: the PDM/DMA writes a garbage value into the FINAL word of
     * every block (measured on hardware: sample [CHUNK_SAMPLES-1] is a
     * near-full-scale outlier in ~70-75% of blocks while every other sample is
     * clean - this is the actual per-chunk click the user heard, NOT a re-arm
     * gap). Hold the previous sample over that last word; one repaired sample per
     * block is inaudible and removes the click. Repairing in a separate buffer
     * (not in place) avoids dirtying a DMA-target cache line, which could
     * otherwise get written back over fresh DMA data when the ring wraps. */
    memcpy(s_chunk_out, src, CHUNK_BYTES);
    s_chunk_out[CHUNK_SAMPLES - 1] = s_chunk_out[CHUNK_SAMPLES - 2];

    if (out_len_bytes) {
        *out_len_bytes = CHUNK_BYTES;
    }
    s_r_count++;
    return s_chunk_out;
}
