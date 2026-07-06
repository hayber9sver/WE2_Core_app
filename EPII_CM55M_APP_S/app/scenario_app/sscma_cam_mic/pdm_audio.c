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

#define CHUNK_SAMPLES   (2048)                 /* samples per chunk (int16, mono) */
#define CHUNK_BYTES     (CHUNK_SAMPLES * 2)    /* bytes per chunk */
#define NUM_BUFF        (4)                    /* ping-pong ring depth */

/* hx_drv_pdm_dma_lli_transfer() hard-requires a 32-byte-aligned buffer: pass
 * an unaligned one and it prints a warning and returns an error *without*
 * starting the transfer, so pdm_audio_rx_cb() never fires and every chunk
 * poll silently returns NULL forever. Confirmed by disassembling the
 * prebuilt libdriver.a - there's no source for it in this tree. */
static int16_t s_audio_buf[NUM_BUFF][CHUNK_SAMPLES] __attribute__((aligned(32)));
static PDM_DEV_INFO s_pdm_dev_info;
static uint32_t s_sample_rate_hz = PDM_AUDIO_DEFAULT_SAMPLE_RATE_HZ;
static volatile int32_t s_w_idx = 0;   /* next buffer the DMA will fill */
static int32_t s_r_idx = 0;            /* next buffer to hand to the caller */
static volatile bool s_running = false;
static bool s_pdm_inited = false;

static void pdm_audio_rx_cb(uint32_t int_status)
{
    (void)int_status;
    int32_t next = (s_w_idx + 1) % NUM_BUFF;
    hx_drv_pdm_dma_lli_transfer((void *)s_audio_buf[next], 1, CHUNK_BYTES, 0);
    s_w_idx = next;
}

static PDM_PCM_FREQ_E rate_to_enum(uint32_t rate_hz)
{
    switch (rate_hz) {
    case 8000:  return PDM_PCM_FREQ_8K;
    case 16000: return PDM_PCM_FREQ_16K;
    case 32000: return PDM_PCM_FREQ_32K;
    case 48000: return PDM_PCM_FREQ_48K;
    default:    return PDM_PCM_FREQ_16K;
    }
}

static void build_cfg(void)
{
    memset(&s_pdm_dev_info, 0, sizeof(s_pdm_dev_info));
    s_pdm_dev_info.pdm_cfg.reg_addr = HIMAX_PDM_BASE_ADDR;
    s_pdm_dev_info.pdm_cfg.rx_fifo_threshold = 5;
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
    if (sample_rate_hz != 8000 && sample_rate_hz != 16000 &&
        sample_rate_hz != 32000 && sample_rate_hz != 48000) {
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
    s_w_idx = 0;
    s_r_idx = 0;
    hx_drv_pdm_dma_lli_transfer((void *)s_audio_buf[0], 1, CHUNK_BYTES, 0);
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
    /* Buffer s_r_idx is fully captured once the DMA has moved past it. */
    if (s_r_idx == s_w_idx) {
        return NULL; /* nothing new yet */
    }
    const int16_t *chunk = s_audio_buf[s_r_idx];
    SCB_InvalidateDCache_by_Addr((uint32_t *)chunk, CHUNK_BYTES);
    if (out_len_bytes) {
        *out_len_bytes = CHUNK_BYTES;
    }
    s_r_idx = (s_r_idx + 1) % NUM_BUFF;
    return chunk;
}
