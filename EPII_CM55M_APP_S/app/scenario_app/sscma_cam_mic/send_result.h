
#ifdef __cplusplus
#include <cstring>
#include <forward_list>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#endif


#include <hx_drv_uart.h>
#include <math.h>
extern "C" {
#include "hx_drv_swreg_aon.h"
}
#include "common_config.h"
#define CONSOLE_UART_ID 0
#define EL_ATTR_WEAK __attribute__((weak))
#define EL_VERSION                 __TIMESTAMP__
#define CONFIG_SSCMA_CMD_MAX_LENGTH (4096)
#define KEYPOINT_NUM 17
#define FM_POINT_NUM 468
#define FM_IRIS_POINT_NUM 10
#define MAX_FACE_LAND_MARK_TRACKED_POINT 68
typedef enum {
    EL_OK      = 0,  // success
    EL_AGAIN   = 1,  // try again
    EL_ELOG    = 2,  // logic error
    EL_ETIMOUT = 3,  // timeout
    EL_EIO     = 4,  // IO error
    EL_EINVAL  = 5,  // invalid argument
    EL_ENOMEM  = 6,  // out of memory
    EL_EBUSY   = 7,  // busy
    EL_ENOTSUP = 8,  // not supported
    EL_EPERM   = 9,  // operation not permitted
} el_err_code_t;
typedef enum {
    EL_PIXEL_FORMAT_RGB888 = 0,
    EL_PIXEL_FORMAT_RGB565,
    EL_PIXEL_FORMAT_YUV422,
    EL_PIXEL_FORMAT_GRAYSCALE,
    EL_PIXEL_FORMAT_JPEG,
    EL_PIXEL_FORMAT_UNKNOWN,
} el_pixel_format_t;
typedef enum el_pixel_rotate_t {
    EL_PIXEL_ROTATE_0 = 0,
    EL_PIXEL_ROTATE_90,
    EL_PIXEL_ROTATE_180,
    EL_PIXEL_ROTATE_270,
    EL_PIXEL_ROTATE_UNKNOWN,
} el_pixel_rotate_t;
typedef struct el_img_t {
    uint8_t*          data;
    size_t            size;
    uint16_t          width;
    uint16_t          height;
    el_pixel_format_t format;
    el_pixel_rotate_t rotate;
} el_img_t;
typedef struct el_box_t {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint8_t  score;
    uint8_t  target;
} el_box_t;

typedef struct el_point_t {
    uint16_t x;
    uint16_t y;
    uint8_t  score;
    uint8_t  target;
} el_point_t;

typedef struct el_class_t {
    uint16_t score;
    uint16_t target;
} el_class_t;

typedef struct el_keypoint_t {
    el_box_t el_box;
    el_point_t el_keypoint[KEYPOINT_NUM];
} el_keypoint_t;


typedef struct {
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
	int16_t MAR;
	int16_t REAR;
	int16_t LEAR;
    int16_t left_iris_theta;
	int16_t left_iris_phi;
	int16_t right_iris_theta;
	int16_t right_iris_phi;
}el_struct_angle;

typedef struct el_fm_point_t {
    el_box_t el_box;
    el_point_t el_fm_point[FM_POINT_NUM];
    el_point_t el_fm_iris[FM_IRIS_POINT_NUM];
    el_struct_angle el_fm_angle;
} el_fm_point_t;


typedef struct el_fd_fl_t {
    el_box_t el_box;
    el_point_t el_fl[MAX_FACE_LAND_MARK_TRACKED_POINT];
    el_struct_angle el_fl_angle;
} el_fd_fl_t;

typedef struct el_fd_fl_el_9pt_t {
    el_box_t el_box;
    el_point_t el_fl[MAX_FACE_LAND_MARK_TRACKED_POINT];
    el_point_t left_eye_landmark[9];
	el_point_t right_eye_landmark[9];
    el_struct_angle el_fl_angle;
} el_fd_fl_el_9pt_t;

/**
 * @brief Algorithm Types
 */
typedef enum {
    EL_ALGO_TYPE_UNDEFINED = 0u,
    EL_ALGO_TYPE_FOMO      = 1u,
    EL_ALGO_TYPE_PFLD      = 2u,
    EL_ALGO_TYPE_YOLO      = 3u,
    EL_ALGO_TYPE_IMCLS     = 4u
} el_algorithm_type_t;
typedef struct el_model_info_t {
    uint8_t             id;
    el_algorithm_type_t type;
    uint32_t            addr_flash;
    uint32_t            size;
    const uint8_t*      addr_memory;
} el_model_info_t;

constexpr inline std::size_t lengthof(const char* s) {
    std::size_t size = 0;
    while (*(s + size) != '\0') ++size;
    return size;
}

template <class T, std::size_t N> constexpr inline std::size_t lengthof(const T (&)[N]) noexcept { return N; }

inline std::size_t lengthof(const std::string& s) { return s.length(); }

// template <typename... Args> constexpr inline decltype(auto) concat_strings(Args&&... args) {
//     std::size_t length{(lengthof(args) + ...)};  // try calculate total length at compile time
//     std::string result;
//     result.reserve(length + 1);  // preallocate memory, avoid repeatly allocate memory while appendings
//     (result.append(std::forward<Args>(args)), ...);
//     return result;
// }

template <typename... Args> constexpr inline decltype(auto) concat_strings(Args&&... args) {
    std::size_t length{(lengthof(args) + ...)};
    std::string result;
    result.reserve(length);
    (result.append(std::forward<Args>(args)), ...);
    return result;
}



el_err_code_t send_bytes(const char* buffer, size_t size) ;
el_err_code_t read_bytes_nonblock(char* buffer, size_t size);
void el_base64_encode(const unsigned char* in, int in_len, char* out);
void event_reply(const std::string& data);
void event_reply_named(const char* name, const std::string& data);
/* Like event_reply_named(), but the largest field (raw bytes, base64-encoded
 * as `field_name`) is streamed to the UART in bounded chunks rather than
 * built as one big heap string first - see send_result.cpp for why. */
void event_reply_named_with_payload(const char* name, const std::string& prefix_fields,
                                     const char* field_name, const uint8_t* raw, size_t raw_len);
/* Dedicated binary (no JSON, no base64) framing for the ASAMPLE audio path -
 * see send_result.cpp for the wire format and why this exists (base64+JSON's
 * ~33%+ size tax ate the entire inter-chunk time budget at 32kHz on the
 * 921600-baud UART, so the PDM ring wrapped under the reader). Magic's first
 * byte is 0xFF, not ASCII, so it can't collide with a coincidental "ASMB"
 * inside a concurrent base64-encoded image payload - see send_result.cpp.
 *
 * 2026-07-10: split from one atomic send_audio_binary_frame() call into
 * begin/pump so at_cmd_poll() can drip a chunk out over many ~5ms polls
 * instead of one ~87ms blocking send_bytes() burst (8018 bytes at 921600
 * baud) held under out_transport_lock() - see send_result.cpp for the
 * hardware symptom that motivated this. */
void audio_tx_begin_frame(const int16_t* pcm, size_t len_bytes, uint32_t sample_rate,
                           uint8_t channels, uint8_t bits);
/* True while a frame started by audio_tx_begin_frame() still has bytes left
 * to send. at_cmd_poll() must not start a new frame, and must not dispatch
 * a newly-completed AT command line, while this is true - either would
 * splice foreign bytes into the middle of the framed payload on the wire.
 *
 * 2026-07-17: extern "C" added specifically so i2c_cmd.c (a plain .c file,
 * doesn't include this C++-oriented header at all) can declare and call
 * these two with a matching, non-name-mangled symbol - its own I2C command
 * dispatch path needed the exact same audio_tx_busy()-drain-before-dispatch
 * guard at_cmd_poll() already had, and skipping it was the root cause of an
 * intermittent "command sent, WE2 replied, ESP32 never sees a valid reply"
 * bug during active audio streaming (see i2c_cmd.c's own comment at its
 * call site for the full story). */
extern "C" {
bool audio_tx_busy(void);
/* Sends one small piece (see AUDIO_TX_PIECE_BYTES in send_result.cpp) of the
 * frame started by audio_tx_begin_frame(). No-op if not busy. */
void audio_tx_pump(void);
}
/* Each sends exactly one Operation Response for its own query, per the AT
 * protocol (at-protocol-en_US.md) - replaces the old send_device_id(), which
 * blasted all four back-to-back regardless of which one was actually asked
 * for and used the wrong "name" field on every one of them. */
void send_name_reply();
void send_ver_reply();
void send_id_reply();
void send_info_reply();
/* Builds the "model" object embedded in AT+INVOKE's Operation Response -
 * AT+MODEL?/AT+MODELS? themselves are out of scope for this app. */
el_model_info_t current_model_info();
std::string model_info_2_json_str(el_model_info_t model_info);
/* Defined inline (not just declared) because it's a template: any
 * translation unit that instantiates it for a new T (e.g. el_class_t from
 * cvapp.cpp) needs the body visible, not just a prototype. */
template <typename T> inline std::string results_2_json_str(const std::forward_list<T>& results) {
    std::string ss;
    const char* delim = "";

    if constexpr (std::is_same<T, el_box_t>::value) {
        ss = "\"boxes\": [";
        for (const auto& box : results) {
            ss += concat_strings(delim,
                                 "[",
                                 std::to_string(box.x),
                                 ", ",
                                 std::to_string(box.y),
                                 ", ",
                                 std::to_string(box.w),
                                 ", ",
                                 std::to_string(box.h),
                                 ", ",
                                 std::to_string(box.score),
                                 ", ",
                                 std::to_string(box.target),
                                 "]");
            delim = ", ";
        }
    } else if constexpr (std::is_same<T, el_point_t>::value) {
        ss = "\"points\": [";
        for (const auto& point : results) {
            ss += concat_strings(delim,
                                 "[",
                                 std::to_string(point.x),
                                 ", ",
                                 std::to_string(point.y),
                                 ", ",
                                 std::to_string(point.score),
                                 ", ",
                                 std::to_string(point.target),
                                 "]");
            delim = ", ";
        }
    } else if constexpr (std::is_same<T, el_class_t>::value) {
        ss = "\"classes\": [";
        for (const auto& cls : results) {
            ss += concat_strings(delim, "[", std::to_string(cls.score), ", ", std::to_string(cls.target), "]");
            delim = ", ";
        }
    }
    ss += "]";

    return ss;
}

std::string  box_results_2_json_str(std::forward_list<el_box_t>& results);
std::string  fm_point_results_2_json_str(std::forward_list<el_fm_point_t>& results);
std::string  keypoint_results_2_json_str(std::forward_list<el_keypoint_t>& results);
std::string  img_res_2_json_str(const el_img_t* img);
std::string  algo_tick_2_json_str(uint32_t algo_tick);
std::string  fd_fl_results_2_json_str(std::forward_list<el_fd_fl_t>& results);
std::string  fd_fl_el_9t_results_2_json_str(std::forward_list<el_fd_fl_el_9pt_t>& results);
std::string  fm_face_bbox_results_2_json_str(std::forward_list<el_box_t>& results);