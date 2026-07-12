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
#include "WE2_device.h"    /* NVIC_SystemReset() for AT+RST */
#include "FreeRTOS.h"
#include "task.h"          /* xTaskGetTickCount() for AT+BREAK's timestamp */
#include "send_result.h"
#include "at_cmd.h"

#define LINE_BUF_SIZE (96)
#define TAG_BUF_SIZE  (24)

static char     s_line[LINE_BUF_SIZE];
static size_t   s_line_len = 0;

/* 2026-07-10: current command's tag (AT+<Tag>@<Body>, at-protocol-en_US.md's
 * "Tagging" section) - see at_cmd.h's comment for why this exists. Reset at
 * the top of every process_line() call; "" when the line had no tag. */
static char     s_cur_tag[TAG_BUF_SIZE] = "";

const char* at_cmd_current_tag(void) { return s_cur_tag; }

static void reply_simple(const char* name, el_err_code_t code, const std::string& data)
{
    const std::string& tagged = s_cur_tag[0] ? concat_strings(s_cur_tag, "@", name) : std::string(name);
    const auto& ss = concat_strings("\r{\"type\": 0, \"name\": \"", tagged.c_str(),
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

/* AT+SENSOR/AT+SENSORS opt table. 2026-07-10: replaced the old 320x240/
 * 160x112 options - those were never a true full-FOV downscale (see
 * cisdp_cfg.h's enum comment), just a top-left crop of the sensor image.
 * These are real *centered* square crops (see cisdp_sensor.c's
 * subs_get_roi()), useful for future square-input models. Indexed by
 * opt_id (0/1/2, largest last to match the AT protocol spec's own
 * AT+SENSORS? example ordering). */
struct sensor_opt_t { int opt_id; int width; int height; const char* detail; APP_DP_INP_SUBSAMPLE_E subs; };
static const sensor_opt_t kSensorOpts[3] = {
    {0, 240, 240, "240x240 Auto", APP_DP_RES_RGB640x480_INP_CROP_240x240},
    {1, 480, 480, "480x480 Auto", APP_DP_RES_RGB640x480_INP_CROP_480x480},
    {2, 640, 480, "640x480 Auto", APP_DP_RES_RGB640x480_INP_SUBSAMPLE_1X},
};
/* Tracks the opt_id AT+SENSOR last requested (default matches g_cam_subs'
 * own default in sscma_cam_mic.c - APP_DP_RES_RGB640x480_INP_SUBSAMPLE_1X).
 * app_request_cam_resolution() is asynchronous (cam_task applies it within
 * the next couple of frames - see its comment), so this reflects the last
 * *requested* resolution, not necessarily what's mid-frame on the wire this
 * instant - same caveat the protocol's own opt_id/opt_detail fields have on
 * any asynchronous sensor. */
static int s_sensor_opt = 2;

static std::string sensor_info_json()
{
    const sensor_opt_t& o = kSensorOpts[s_sensor_opt];
    return concat_strings("{\"id\": 1, \"type\": 1, \"state\": 2, \"opt_id\": ",
                           std::to_string(o.opt_id), ", \"opt_detail\": \"", o.detail, "\"}");
}

/* 2026-07-10: cmd_invoke/cmd_sample/cmd_asample used to all reply with
 * send_device_id() - a leftover placeholder that blasted NAME?/VER?/ID?/
 * INFO?/MODEL? back-to-back regardless of which command was actually sent,
 * with the wrong "name" field on every one of them (e.g. AT+INVOKE's first
 * reply claimed to be "NAME?"). The AT protocol (at-protocol-en_US.md)
 * requires each Execute operation to reply with its OWN correctly-named
 * Operation Response; the actual per-frame results still go out separately
 * via event_reply_named("INVOKE"/"SAMPLE", ...) in cvapp.cpp, unaffected by
 * this fix - capture_cam_mic.py already only reads those event replies, not
 * this one, so this was a silent protocol violation rather than something
 * that broke the existing tooling. */
static void cmd_invoke(const char* args)
{
    char buf[LINE_BUF_SIZE];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    long result_only = parse_nth_int(buf, 2, 0);

    app_set_result_only(result_only != 0);
    app_set_cam_stream_mode(CAM_STREAM_INVOKE);

    std::string data = concat_strings(
        "{\"model\": ", model_info_2_json_str(current_model_info()),
        ", \"algorithm\": {\"type\": 3, \"category\": 1, \"input_from\": 1, \"config\": {",
        "\"tscore\": ", std::to_string(app_get_tscore()),
        ", \"tiou\": ", std::to_string(app_get_tiou()), "}}",
        ", \"sensor\": ", sensor_info_json(), "}");
    reply_simple("INVOKE", EL_OK, data);
}

static void cmd_sample(const char*)
{
    app_set_cam_stream_mode(CAM_STREAM_SAMPLE);
    reply_simple("SAMPLE", EL_OK, concat_strings("{\"sensor\": ", sensor_info_json(), "}"));
}

/* Not in the AT protocol spec (audio sampling has no equivalent there) -
 * app-specific extension, mirrors AT+SAMPLE's reply shape. */
static void cmd_asample(const char*)
{
    if (!pdm_audio_is_running()) {
        pdm_audio_start();
    }
    app_set_audio_streaming(1);
    reply_simple("ASAMPLE", EL_OK,
                 concat_strings("{\"sample_rate\": ", std::to_string(pdm_audio_get_rate()), "}"));
}

/* 2026-07-10: added for symmetry with the spec's own AT+SAMPLE?/AT+INVOKE?
 * ("Get sample status"/"Get invoke status" - bare 0/1, not an object) - this
 * app previously had no way to ask "is audio currently streaming?" short of
 * tracking client-side state yourself. */
static void cmd_asample_query()
{
    reply_simple("ASAMPLE?", EL_OK, app_get_audio_streaming() ? "1" : "0");
}

static void cmd_break(const char*)
{
    /* Stops both streams - there's no separate "ABREAK", BREAK is the
     * universal stop, same as real SSCMA firmware. */
    app_set_cam_stream_mode(CAM_STREAM_IDLE);
    app_set_audio_streaming(0);
    pdm_audio_stop();
    /* Spec's "data" field is a timestamp (device uptime in ms) - see
     * at-protocol-en_US.md's "Stop all running tasks" example. */
    reply_simple("BREAK", EL_OK,
                 std::to_string((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS)));
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

/* NMS IoU threshold - drives cvapp.cpp's run_nms() (see app_get_tiou()'s
 * declaration in sscma_cam_mic.h). Added 2026-07-10 alongside TSCORE so both
 * detection knobs are tunable live instead of only one of them. */
static void cmd_tiou(const char* args, bool is_query)
{
    if (is_query) {
        reply_simple("TIOU?", EL_OK, std::to_string(app_get_tiou()));
        return;
    }
    long v = strtol(args, nullptr, 10);
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    app_set_tiou((uint8_t)v);
    reply_simple("TIOU", EL_OK, std::to_string(app_get_tiou()));
}

/* RGB640x480_* (not YUV640x480_*): the YOLO detect model needs a 3-channel
 * RGB raw buffer, not the grayscale-Y-plane-only YUV420 the old person-
 * detect model used - see cisdp_sensor.c's demosbuf sizing. */
static void cmd_sensor(const char* args)
{
    char buf[LINE_BUF_SIZE];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    long opt = parse_nth_int(buf, 2, -1);

    if (opt < 0 || opt > 2) {
        reply_simple("SENSOR", EL_EINVAL, "\"opt_id must be 0 (240x240), 1 (480x480), or 2 (640x480)\"");
        return;
    }

    s_sensor_opt = (int)opt;
    app_request_cam_resolution((int)kSensorOpts[s_sensor_opt].subs);
    reply_simple("SENSOR", EL_OK, concat_strings("{\"sensor\": ", sensor_info_json(), "}"));
}

static void cmd_sensor_query()
{
    reply_simple("SENSOR?", EL_OK, sensor_info_json());
}

static void cmd_sensors_query()
{
    const sensor_opt_t& cur = kSensorOpts[s_sensor_opt];
    std::string opts = concat_strings(
        "{\"2\": \"", kSensorOpts[2].detail, "\"",
        ", \"1\": \"", kSensorOpts[1].detail, "\"",
        ", \"0\": \"", kSensorOpts[0].detail, "\"}");
    reply_simple("SENSORS?", EL_OK,
                 concat_strings("{\"id\": 1, \"type\": 1, \"state\": 2, \"opt_id\": ",
                                std::to_string(cur.opt_id), ", \"opt_detail\": \"", cur.detail,
                                "\", \"opts\": ", opts, "}"));
}

static void cmd_stat_query()
{
    reply_simple("STAT?", EL_OK,
                 concat_strings("{\"boot_count\": ", std::to_string(app_get_boot_count()),
                                ", \"is_ready\": 1}"));
}

static void cmd_rst()
{
    /* No-reply per protocol (Reserved operation, see "Reboot device" in
     * at-protocol-en_US.md) - never returns. */
    NVIC_SystemReset();
}

static void cmd_help()
{
    /* Raw text, not a JSON envelope - matches the protocol's own AT+HELP?
     * example, which is a plain command list rather than an Operation
     * Response. */
    static const char help_text[] =
        "Command list:\n"
        "  AT+INVOKE=<n_times,differed,result_only>  camera + YOLO gesture inference loop\n"
        "  AT+SAMPLE=<n_times>                       raw camera stream, no inference\n"
        "  AT+ASAMPLE=<n_times>/AT+ASAMPLE?          start/query raw DMIC/PDM audio stream (binary frames)\n"
        "  AT+BREAK                                  stop whichever stream is active\n"
        "  AT+ASR=<rate>/AT+ASR?                     set/query the PDM sample rate\n"
        "  AT+TSCORE=<0..100>/AT+TSCORE?             detection score threshold\n"
        "  AT+TIOU=<0..100>/AT+TIOU?                 NMS IoU threshold\n"
        "  AT+SENSOR=<id,enable,opt_id>/AT+SENSOR?   set/query camera resolution\n"
        "  AT+SENSORS?                               list available camera resolutions\n"
        "  AT+STAT?                                  boot count / ready state\n"
        "  AT+ID?/AT+NAME?/AT+VER?/AT+INFO?          device identity\n"
        "  AT+RST                                    reboot\n"
        "  AT+HELP?                                  this list\n";
    send_bytes(help_text, sizeof(help_text) - 1);
}

static void process_line(char* line)
{
    if (strncmp(line, "AT+", 3) != 0) {
        return; /* not an AT command line, ignore (e.g. stray CR/LF) */
    }
    char* name = line + 3;

    /* Tag support (AT+<Tag>@<Body>, at-protocol-en_US.md's "Tagging"
     * section) - see at_cmd.h's at_cmd_current_tag() comment for why this
     * matters (the ESP32C3 camera_web_server bridge tags every command it
     * relays and needs the tag echoed back to correlate replies). The '@'
     * only counts as a tag delimiter if it appears before both '='/'?' - a
     * command's own name never legitimately contains '@', but a quoted
     * argument value theoretically could (e.g. AT+INFO="foo@bar"), and that
     * must not be mistaken for a tag. */
    s_cur_tag[0] = '\0';
    {
        char* at = strchr(name, '@');
        char* eq_probe = strchr(name, '=');
        char* q_probe  = strchr(name, '?');
        if (at != nullptr && (eq_probe == nullptr || at < eq_probe) && (q_probe == nullptr || at < q_probe)) {
            size_t tag_len = (size_t)(at - name);
            if (tag_len < sizeof(s_cur_tag)) {
                memcpy(s_cur_tag, name, tag_len);
                s_cur_tag[tag_len] = '\0';
                name = at + 1;
            }
            /* else: tag too long for the scratch buffer - fall through and
             * treat the whole "<tag>@<body>" run as one (unrecognized)
             * command name, same as if no tag delimiter had been found. */
        }
    }

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
        if (is_query) cmd_asample_query();
        else cmd_asample(args);
    } else if (strcmp(name, "BREAK") == 0) {
        cmd_break(args);
    } else if (strcmp(name, "ASR") == 0) {
        cmd_asr(args, is_query);
    } else if (strcmp(name, "TSCORE") == 0) {
        cmd_tscore(args, is_query);
    } else if (strcmp(name, "TIOU") == 0) {
        cmd_tiou(args, is_query);
    } else if (strcmp(name, "SENSORS") == 0) {
        if (is_query) cmd_sensors_query();
        else reply_simple("SENSORS", EL_ENOTSUP, "\"read-only\"");
    } else if (strcmp(name, "SENSOR") == 0) {
        if (is_query) cmd_sensor_query();
        else cmd_sensor(args);
    } else if (strcmp(name, "STAT") == 0) {
        if (is_query) cmd_stat_query();
        else reply_simple("STAT", EL_ENOTSUP, "\"read-only\"");
    } else if (strcmp(name, "RST") == 0) {
        cmd_rst();
    } else if (strcmp(name, "HELP") == 0) {
        cmd_help();
    } else if (is_query && strcmp(name, "NAME") == 0) {
        send_name_reply();
    } else if (is_query && strcmp(name, "VER") == 0) {
        send_ver_reply();
    } else if (is_query && strcmp(name, "ID") == 0) {
        send_id_reply();
    } else if (is_query && strcmp(name, "INFO") == 0) {
        send_info_reply();
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
    uint32_t rx_spins = 0;
    while (read_bytes_nonblock(&c, 1) == EL_OK) {
        got_byte = true;
        /* Defense-in-depth (kept permanently, like the idle-flush below):
         * should the DW_UART driver ever wedge into reporting RX-ready
         * forever, this loop would otherwise spin unbounded - and since
         * audio_task outranks cam_task, that would starve the camera
         * silently. Bail out and report instead. Legitimate traffic can't
         * hit this: 921600 baud delivers at most ~460 bytes per 5ms poll. */
        if (++rx_spins > 512u) {
            xprintf("at_cmd: runaway RX loop, last byte=0x%02x\n",
                    (unsigned char)c);
            s_line_len = 0;
            break;
        }
        if (c == '\r' || c == '\n') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                /* An audio frame's bytes are being drip-fed across many polls
                 * by audio_tx_pump() (see below) - dispatching a command here
                 * mid-frame would send its reply via send_bytes() in between
                 * two pieces, splicing foreign bytes into the middle of the
                 * framed payload on the wire. Drain the rest of the current
                 * frame first; it's at most a few hundred us worth of pieces
                 * away from done. */
                while (audio_tx_busy()) {
                    audio_tx_pump();
                }
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
        while (audio_tx_busy()) {
            audio_tx_pump();
        }
        process_line(s_line);
        s_line_len = 0;
        s_idle_polls = 0;
    }

    /* Drip the current audio frame (if any) out a small piece at a time
     * rather than blocking this whole poll on one ~87ms send_bytes() burst -
     * see audio_tx_begin_frame()'s comment in send_result.h. Only start a
     * *new* frame once the previous one has fully drained (audio_tx_busy()
     * false); audio_tx keeps draining even after AT+BREAK clears
     * app_get_audio_streaming(), so a frame already in flight always finishes
     * intact instead of being cut off mid-payload. */
    if (audio_tx_busy()) {
        audio_tx_pump();
    } else if (app_get_audio_streaming()) {
        uint32_t len = 0;
        const int16_t* chunk = pdm_audio_poll_chunk(&len);
        if (chunk != nullptr) {
            audio_tx_begin_frame(chunk, len, pdm_audio_get_rate(), 1, 16);
            audio_tx_pump();
        }
    }
}
