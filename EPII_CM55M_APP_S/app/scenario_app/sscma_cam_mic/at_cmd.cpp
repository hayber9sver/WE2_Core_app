/*
 * at_cmd.cpp
 *
 * See at_cmd.h. Written from scratch (no SSCMA parser ships in this repo -
 * confirmed by grepping the tree for the literal AT+ command strings
 * before starting this app) to match what himax_vision.py sends, plus a
 * couple of app-specific audio commands.
 */

extern "C" {
#include <string.h>
#include <stdlib.h>
#include "sscma_cam_mic.h"
#include "pdm_audio.h"
#include "xprintf.h"
#include "cisdp_cfg.h"
}
#include <cstdint>
#include <string>
#include "send_result.h"
#include "at_cmd.h"

#define LINE_BUF_SIZE (96)

static char     s_line[LINE_BUF_SIZE];
static size_t   s_line_len = 0;

static void reply_simple(const char* name, el_err_code_t code, const std::string& data)
{
    const auto& ss = concat_strings("\r{\"type\": 0, \"name\": \"", name,
                                     "\", \"code\": ", std::to_string((int)code),
                                     ", \"data\": ", data, "}\n");
    send_bytes(ss.c_str(), ss.size());
}

static long parse_nth_int(char* args, int index, long fallback)
{
    /* args is mutable scratch (a copy), split on ',' */
    char* tok = strtok(args, ",");
    for (int i = 0; tok != nullptr && i < index; ++i) {
        tok = strtok(nullptr, ",");
    }
    return tok ? strtol(tok, nullptr, 10) : fallback;
}

static void cmd_invoke(const char* args)
{
    char buf[LINE_BUF_SIZE];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    long result_only = parse_nth_int(buf, 2, 0);

    app_set_result_only(result_only != 0);
    app_set_cam_stream_mode(CAM_STREAM_INVOKE);
    send_device_id();
}

static void cmd_sample(const char*)
{
    app_set_cam_stream_mode(CAM_STREAM_SAMPLE);
    send_device_id();
}

static void cmd_asample(const char*)
{
    if (!pdm_audio_is_running()) {
        pdm_audio_start();
    }
    app_set_audio_streaming(1);
    send_device_id();
}

static void cmd_break(const char*)
{
    /* Stops both streams - there's no separate "ABREAK", BREAK is the
     * universal stop, same as real SSCMA firmware. */
    app_set_cam_stream_mode(CAM_STREAM_IDLE);
    app_set_audio_streaming(0);
    pdm_audio_stop();
}

static void cmd_asr(const char* args, bool is_query)
{
    if (is_query) {
        reply_simple("ASR?", EL_OK,
                     concat_strings("{\"sample_rate\": ", std::to_string(pdm_audio_get_rate()), "}"));
        return;
    }
    long rate = strtol(args, nullptr, 10);
    int ret = pdm_audio_set_rate((uint32_t)rate);
    reply_simple("ASR", ret == 0 ? EL_OK : EL_EINVAL,
                 concat_strings("{\"sample_rate\": ", std::to_string(pdm_audio_get_rate()), "}"));
}

static void cmd_tscore(const char* args, bool is_query)
{
    if (is_query) {
        reply_simple("TSCORE?", EL_OK, std::to_string(app_get_tscore()));
        return;
    }
    long v = strtol(args, nullptr, 10);
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    app_set_tscore((uint8_t)v);
    reply_simple("TSCORE", EL_OK, std::to_string(app_get_tscore()));
}

static void cmd_sensor(const char* args)
{
    char buf[LINE_BUF_SIZE];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    long opt = parse_nth_int(buf, 2, -1);

    /* OV5647's datapath only has three real digital-downscale sizes off its
     * native 640x480 binned output (1X/2X/4X subsample) - there is no 240x240
     * option, and nothing larger without disabling binning entirely (a
     * different sensor init table, out of scope here). opt=2/1/0 map to
     * 640x480/320x240/160x112, matching the SENSOR-resolution ordering used
     * elsewhere in this protocol (largest first).
     *
     * opt=0 is 160x112, not the "native" 160x120 4X subsample: the JPEG
     * encoder on this SoC requires both dimensions to be a multiple of 16
     * (confirmed on hardware - 120 isn't, and produced a permanent
     * EVT_INDEX_EDM_WDT2_TIMEOUT with zero frames ever coming out), so
     * cisdp_dp_init() crops the encoded height down to 112 (real captured
     * rows, not padding) to satisfy that. app_get_raw_height() already
     * reflects this, so img_rescale()'s NPU input scaling still lines up. */
    int width, height, subs;
    switch (opt) {
    case 2: width = 640; height = 480; subs = APP_DP_RES_YUV640x480_INP_SUBSAMPLE_1X; break;
    case 1: width = 320; height = 240; subs = APP_DP_RES_YUV640x480_INP_SUBSAMPLE_2X; break;
    case 0: width = 160; height = 112; subs = APP_DP_RES_YUV640x480_INP_SUBSAMPLE_4X; break;
    default:
        reply_simple("SENSOR", EL_EINVAL, "\"opt must be 0 (160x112), 1 (320x240), or 2 (640x480)\"");
        return;
    }

    app_request_cam_resolution(subs);
    reply_simple("SENSOR", EL_OK,
                 concat_strings("{\"resolution\": [", std::to_string(width), ", ", std::to_string(height), "]}"));
}

static void process_line(char* line)
{
    if (strncmp(line, "AT+", 3) != 0) {
        return; /* not an AT command line, ignore (e.g. stray CR/LF) */
    }
    char* name = line + 3;

    char* eq = strchr(name, '=');
    char* q  = strchr(name, '?');
    bool is_query = false;
    char* args = const_cast<char*>("");

    if (eq != nullptr && (q == nullptr || eq < q)) {
        *eq = '\0';
        args = eq + 1;
    } else if (q != nullptr) {
        *q = '\0';
        is_query = true;
    }

    if (strcmp(name, "INVOKE") == 0) {
        cmd_invoke(args);
    } else if (strcmp(name, "SAMPLE") == 0) {
        cmd_sample(args);
    } else if (strcmp(name, "ASAMPLE") == 0) {
        cmd_asample(args);
    } else if (strcmp(name, "BREAK") == 0) {
        cmd_break(args);
    } else if (strcmp(name, "ASR") == 0) {
        cmd_asr(args, is_query);
    } else if (strcmp(name, "TSCORE") == 0) {
        cmd_tscore(args, is_query);
    } else if (strcmp(name, "SENSOR") == 0) {
        cmd_sensor(args);
    } else if (is_query && (strcmp(name, "INFO") == 0 || strcmp(name, "NAME") == 0 ||
                             strcmp(name, "VER") == 0 || strcmp(name, "ID") == 0 ||
                             strcmp(name, "MODEL") == 0)) {
        send_device_id();
    } else {
        reply_simple(name, EL_EINVAL, "\"unsupported command\"");
    }
}

/* at_cmd_poll() runs roughly every 5ms (see audio_task's vTaskDelay). Under
 * heavy camera+NPU load a received byte can occasionally go missing (seen on
 * hardware: a command's trailing \r vanished, so the parser sat waiting
 * forever for a terminator that never showed up, stalling the whole link
 * until some *later* command's own \r happened to flush both as one garbled
 * line). Rather than depend on eliminating that hardware-level byte loss,
 * self-heal here: if a partial line has been sitting idle (no new byte) for
 * this many polls, process whatever was accumulated as-is. */
#define LINE_IDLE_FLUSH_POLLS (10) /* ~50ms of silence with a pending partial line */
static uint32_t s_idle_polls = 0;

void at_cmd_init(void)
{
    s_line_len = 0;
    s_idle_polls = 0;
}

void at_cmd_poll(void)
{
    char c;
    bool got_byte = false;
    while (read_bytes_nonblock(&c, 1) == EL_OK) {
        got_byte = true;
        if (c == '\r' || c == '\n') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                process_line(s_line);
                s_line_len = 0;
            }
        } else if (s_line_len < LINE_BUF_SIZE - 1) {
            s_line[s_line_len++] = c;
        } else {
            s_line_len = 0; /* overflow: drop the line */
        }
    }

    if (got_byte) {
        s_idle_polls = 0;
    } else if (s_line_len > 0 && ++s_idle_polls >= LINE_IDLE_FLUSH_POLLS) {
        s_line[s_line_len] = '\0';
        xprintf("at_cmd: idle-flushing pending line (lost terminator?): '%s'\r\n", s_line);
        process_line(s_line);
        s_line_len = 0;
        s_idle_polls = 0;
    }

    if (app_get_audio_streaming()) {
        uint32_t len = 0;
        const int16_t* chunk = pdm_audio_poll_chunk(&len);
        if (chunk != nullptr) {
            /* Streamed instead of audio_2_json_str()+concat_strings() - see
             * event_reply_named_with_payload()'s comment in send_result.cpp;
             * this buffer no longer needs a big heap allocation either. */
            event_reply_named_with_payload("ASAMPLE",
                concat_strings(", \"sample_rate\": ", std::to_string(pdm_audio_get_rate()),
                               ", \"channels\": 1, \"bits\": 16"),
                "audio", reinterpret_cast<const uint8_t*>(chunk), len);
        }
    }
}
