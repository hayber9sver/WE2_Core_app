/*
 * i2c_cmd.c
 *
 * See i2c_cmd.h. Pinmux + init sequence copied from hx_drv_iic.h's own
 * "Sample code: I2C slave 0 pin mux configuration and initialization" /
 * "Usage-2: Receive data using interrupt mode with I2C slave 0" doc comment
 * (hx_drv_iic.h ~line 125-198) - that's the lowest-level, most directly
 * documented path (hx_drv_i2cs_init/hx_drv_i2cs_interrupt_read), deliberately
 * NOT library/i2c_comm's hx_lib_i2ccomm_* wrapper (that one imposes its own
 * binary [feature][cmd][len][payload][checksum] framing, which would force
 * the ESP32 side to build/send a matching frame for no benefit here - we
 * just want raw AT command bytes, same as the UART path already carries).
 * Slave address 0x62 matches both hx_drv_iic.h's own example and
 * Seeed_Arduino_SSCMA.h's I2C_ADDRESS default.
 */

#include <stdbool.h>
#include <string.h>
#include "hx_drv_scu.h"
#include "hx_drv_iic.h"
#include "WE2_device_addr.h" /* HX_I2C_HOST_SLV_0_BASE */
#include "xprintf.h"
#include "at_cmd.h"
#include "i2c_cmd.h"

/* Declared (with C linkage - see send_result.cpp's own extern "C" wrapper)
 * in send_result.h, which this file doesn't include directly (it's a
 * C++-oriented header - STL includes, etc. - not worth pulling into a plain
 * .c file just for these two). See i2c_cmd_poll()'s own comment on why they
 * matter here. */
bool audio_tx_busy(void);
void audio_tx_pump(void);

#define I2C_CMD_SLV_ADDR   (0x62)
#define I2C_CMD_RX_BUF_SIZE (128) /* >= at_cmd.cpp's own LINE_BUF_SIZE (96) */

static uint8_t          s_rx_buf[I2C_CMD_RX_BUF_SIZE];
static volatile uint32_t s_rx_len     = 0;
static volatile bool     s_rx_pending = false;

/* ISR context (interrupt mode driver callback, matches the pattern
 * allon_sensor_tflm_freertos/comm_task.c's i2cs_cb_rx uses for the same
 * peripheral) - keep this to volatile scalar writes only, no driver calls,
 * no xprintf; actual dispatch happens later from i2c_cmd_poll(). */
static void i2c_cmd_rx_cb(void *param)
{
    HX_DRV_DEV_IIC      *iic_obj  = (HX_DRV_DEV_IIC *)param;
    HX_DRV_DEV_IIC_INFO *iic_info = &(iic_obj->iic_info);

    s_rx_len     = iic_info->rx_buf.ofs;
    s_rx_pending = true;
}

static void i2c_cmd_err_cb(void *param)
{
    (void)param;
    xprintf("i2c_cmd: err_cb fired\r\n");
}

void i2c_cmd_init(void)
{
    /* PA2/PA3 - the WE2 Grove Vision AI module's I2C slave 0 pins (see
     * allon_sensor_tflm_freertos/pinmux_cfg.c's i2cs0_pinmux_cfg(), the only
     * in-repo user of this pin pair, and hx_drv_iic.h's own sample code -
     * both agree on PA2=SCL/PA3=SDA. Unlike out_transport.c's UART1 pin
     * choice, this hasn't been independently checked against the board
     * schematic this session - if I2C traffic never arrives, check this
     * first before suspecting the rest of the chain). */
    hx_drv_scu_set_PA2_pinmux(SCU_PA2_PINMUX_SB_I2C_S_SCL_0, 1);
    hx_drv_scu_set_PA3_pinmux(SCU_PA3_PINMUX_SB_I2C_S_SDA_0, 1);

    /* Not HX_I2C_HOST_SLV_0_BASE: that macro is guarded by #ifdef
     * IP_INST_IIC_SLAVE0 (WE2_device_addr.h) but this app's build defines
     * IP_INST_IIIC_SLAVE0 (extra I - drv_user_defined.mk's DRIVERS_IP_INSTANCE
     * naming doesn't match the device-addr header's guard), so the macro is
     * never defined here - confirmed build error. BASE_ADDR_APB_I2C_SLAVE is
     * the same address, unconditionally defined, no such mismatch. */
    IIC_ERR_CODE_E ret = hx_drv_i2cs_init(USE_DW_IIC_SLV_0, BASE_ADDR_APB_I2C_SLAVE);
    if (ret != IIC_ERR_OK) {
        xprintf("i2c_cmd: hx_drv_i2cs_init FAILED, ret=%d\r\n", (int)ret);
        return;
    }

    hx_drv_i2cs_set_err_cb(USE_DW_IIC_SLV_0, (void *)i2c_cmd_err_cb);

    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    ret = hx_drv_i2cs_interrupt_read(USE_DW_IIC_SLV_0, I2C_CMD_SLV_ADDR, s_rx_buf,
                                      sizeof(s_rx_buf), (void *)i2c_cmd_rx_cb);
    if (ret != IIC_ERR_OK) {
        xprintf("i2c_cmd: hx_drv_i2cs_interrupt_read (initial arm) FAILED, ret=%d\r\n", (int)ret);
        return;
    }

    xprintf("i2c_cmd: I2C slave 0 started OK, addr=0x%02x, SDA=PA3/SCL=PA2\r\n", I2C_CMD_SLV_ADDR);
}

void i2c_cmd_poll(void)
{
    if (!s_rx_pending) {
        return;
    }
    s_rx_pending = false;
    uint32_t len = s_rx_len;

    if (len > 0 && len <= sizeof(s_rx_buf)) {
        /* Same convention as at_cmd.cpp's UART line parser: scan for a
         * '\r'/'\n' terminator rather than trusting any declared length
         * field - there isn't one here, this is raw bytes. */
        char     line[I2C_CMD_RX_BUF_SIZE];
        uint32_t copy_len = 0;
        for (uint32_t i = 0; i < len && copy_len < sizeof(line) - 1; i++) {
            char c = (char)s_rx_buf[i];
            if (c == '\r' || c == '\n') {
                break;
            }
            line[copy_len++] = c;
        }
        line[copy_len] = '\0';
        if (copy_len > 0) {
            /* 2026-07-17: ROOT CAUSE of a hard-to-reproduce bug where
             * commands sent over I2C during active audio streaming would
             * intermittently never get a reply the ESP32 could recognize
             * (sendTaggedCommand() timing out for many consecutive seconds,
             * despite WE2 having genuinely processed the command) - this
             * dispatch point was missing the exact safeguard at_cmd.cpp's
             * own UART-triggered dispatch already has (see its at_cmd_poll(),
             * right before its own process_line() calls, and audio_tx_busy()'s
             * own doc comment in send_result.h): dispatching a command while
             * an audio frame is mid-transmission (drip-fed piece by piece
             * across polls by audio_tx_pump()) splices the command's reply
             * bytes into the middle of that frame's binary payload on the
             * UART1 wire. The reply is still "sent" from WE2's perspective,
             * but arrives corrupted/unparseable at the ESP32 - it can't find
             * the tagged reply, times out, and (before the ESP32-side
             * resync-suppression fix) could even wrongly conclude WE2 had
             * rebooted. Confirmed via dual WE2+ESP32 console capture:
             * AT+SENSOR failed 8 consecutive times while WE2's own
             * AUDIO_POOL kept reporting "both pool slots busy" (audio
             * actively streaming) - i.e. exactly the mid-frame-collision
             * window this drains before dispatching. */
            while (audio_tx_busy()) {
                audio_tx_pump();
            }
            at_cmd_process_line(line);
        }
    }

    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    IIC_ERR_CODE_E ret = hx_drv_i2cs_interrupt_read(USE_DW_IIC_SLV_0, I2C_CMD_SLV_ADDR, s_rx_buf,
                                                     sizeof(s_rx_buf), (void *)i2c_cmd_rx_cb);
    if (ret != IIC_ERR_OK) {
        xprintf("i2c_cmd: hx_drv_i2cs_interrupt_read (re-arm) FAILED, ret=%d\r\n", (int)ret);
    }
}
