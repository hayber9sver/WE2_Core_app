/*
 * cvapp.cpp
 *
 * Ethos-U55 + TFLite-Micro YOLO-style object-detect inference (192x192x3
 * RGB in, [N,8]=[x,y,w,h,objectness,class0,class1,class2] out - SSCMA
 * Swift-YOLO 192, 3-class gesture detection, see decode_boxes()), flash-
 * loaded (FLASH_XIP_MODEL=1, see common_config.h). Replaces the earlier
 * compiled-in 96x96 grayscale person-detect model. cam_handle_frame() also
 * owns turning the result into an SSCMA-style JSON/base64 reply (see
 * send_result.h), gated by the stream mode at_cmd.cpp last set - that
 * keeps all the C++ (std::string/forward_list) usage inside this
 * translation unit, out of the plain-C frame-ready event handler in
 * sscma_cam_mic.c.
 */

#include <cstdio>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <forward_list>
#include <string>
#include "WE2_device.h"
#include "board.h"
#include "cvapp.h"
#include "cisdp_sensor.h"

#include "WE2_core.h"
#include "WE2_device.h"

#include "ethosu_driver.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#include "xprintf.h"
#include "cisdp_cfg.h"

#include "img_proc_helium.h"
#include "common_config.h"
#include "send_result.h"
#include "sscma_cam_mic.h"

#define MODEL_INPUT_W 192
#define MODEL_INPUT_H 192
#define MODEL_INPUT_C 3

#ifdef TRUSTZONE_SEC
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE	BASE_ADDR_APB_U55_CTRL
#endif
#endif

/* tflm_yolov8_od's comparable 192x192 model uses 1053*1024, but this app
 * also statically carries the RGB raw buffer (640x480x3 = 921600B) and the
 * PDM audio ring (32000B) that tflm_yolov8_od doesn't - 1053KB together with
 * those overflowed CM55M_S_SRAM by ~191KB at link time (confirmed on a real
 * build). Trimmed to fit with ~60KB of headroom; if AllocateTensors() fails
 * once the real model is flashed (arena too small), this is the first knob
 * to raise back up - see the SRAM budget math in this app's README. */
#define TENSOR_ARENA_BUFSIZE  (800*1024)
__attribute__(( section(".bss.NoInit"))) uint8_t tensor_arena_buf[TENSOR_ARENA_BUFSIZE] __ALIGNED(32);

using namespace std;

namespace {

constexpr int tensor_arena_size = TENSOR_ARENA_BUFSIZE;
uint32_t tensor_arena = (uint32_t)tensor_arena_buf;

struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */
tflite::MicroInterpreter *int_ptr=nullptr;
TfLiteTensor* input, *output;
};


static void _arm_npu_irq_handler(void)
{
    /* Call the default interrupt handler from the NPU driver */
    ethosu_irq_handler(&ethosu_drv);
}

/**
 * @brief  Initialises the NPU IRQ
 **/
static void _arm_npu_irq_init(void)
{
    const IRQn_Type ethosu_irqnum = (IRQn_Type)U55_IRQn;

    /* Register the EthosU IRQ handler in our vector table.
     * Note, this handler comes from the EthosU driver */
    EPII_NVIC_SetVector(ethosu_irqnum, (uint32_t)_arm_npu_irq_handler);

    /* Enable the IRQ */
    NVIC_EnableIRQ(ethosu_irqnum);

}

static int _arm_npu_init(bool security_enable, bool privilege_enable)
{
    int err = 0;

    /* Initialise the IRQ */
    _arm_npu_irq_init();

    /* Initialise Ethos-U55 device */
    const void * ethosu_base_address = (void *)(U55_BASE);

    if (0 != (err = ethosu_init(
                            &ethosu_drv,             /* Ethos-U driver device pointer */
                            ethosu_base_address,     /* Ethos-U NPU's base address. */
                            NULL,       /* Pointer to fast mem area - NULL for U55. */
                            0, /* Fast mem region size. */
							security_enable,                       /* Security enable. */
							privilege_enable))) {                   /* Privilege enable. */
    	xprintf("failed to initalise Ethos-U device\n");
            return err;
        }

    xprintf("Ethos-U55 device initialised\n");

    return 0;
}

int cv_init(bool security_enable, bool privilege_enable)
{
	int ercode = 0;

	if(_arm_npu_init(security_enable, privilege_enable)!=0)
		return -1;

	/* FLASH_XIP_MODEL is always 1 for this app now (common_config.h) - the
	 * model is xmodem-flashed to MODEL_FLASH_ADDR separately from the
	 * firmware image, not compiled in (see README). */
	/* Diagnostic: dump the raw bytes actually sitting at MODEL_FLASH_ADDR
	 * before letting the flatbuffer parser near them - a version() mismatch
	 * alone can't tell us whether the xmodem burn wrote nothing (flash
	 * stays erased, reads back 0xFF), wrote something at the wrong offset,
	 * or wrote the model but GetModel() is reading it from the wrong
	 * address entirely. Safe to remove once model loading is confirmed
	 * working. */
	{
		const uint8_t *raw = (const uint8_t *)MODEL_FLASH_ADDR;
		xprintf("[model dump] first 32 bytes @ 0x%08x:\n", (unsigned)MODEL_FLASH_ADDR);
		for (int i = 0; i < 32; i += 8) {
			xprintf("  %02x %02x %02x %02x %02x %02x %02x %02x\n",
				raw[i], raw[i+1], raw[i+2], raw[i+3],
				raw[i+4], raw[i+5], raw[i+6], raw[i+7]);
		}
	}

	static const tflite::Model*model = tflite::GetModel((const void *)MODEL_FLASH_ADDR);

	if (model->version() != TFLITE_SCHEMA_VERSION) {
		xprintf(
			"[ERROR] model's schema version %d is not equal "
			"to supported version %d\n",
			model->version(), TFLITE_SCHEMA_VERSION);
		return -1;
	}
	else {
		xprintf("model's schema version %d\n", model->version());
	}

	static tflite::MicroErrorReporter micro_error_reporter;
	static tflite::MicroMutableOpResolver<4> op_resolver;

	/* AddEthosU() covers the NPU-delegated bulk of the graph; AddReshape()/
	 * AddPadV2()/AddTranspose() are CPU-side ops at TFLite-micro graph
	 * boundaries for Ethos-U-delegated models in this SDK (see tflm_fd_fm,
	 * tflm_yolov8_od - the latter's own resolver adds Transpose too, for
	 * the same reason: see CHANGE_YOLOV8_OB_OUPUT_SHAPE in its cvapp).
	 * PadV2 and Transpose both confirmed needed on hardware via
	 * AllocateTensors() "Didn't find op for builtin opcode" errors. If it
	 * fails again with a different missing op, add it here too (bump the
	 * <4> to match). */
	if (kTfLiteOk != op_resolver.AddEthosU()){
		xprintf("Failed to add Arm NPU support to op resolver.");
		return false;
	}
	if (kTfLiteOk != op_resolver.AddReshape()){
		xprintf("Failed to add Reshape support to op resolver.");
		return false;
	}
	if (kTfLiteOk != op_resolver.AddPadV2()){
		xprintf("Failed to add PadV2 support to op resolver.");
		return false;
	}
	if (kTfLiteOk != op_resolver.AddTranspose()){
		xprintf("Failed to add Transpose support to op resolver.");
		return false;
	}

	static tflite::MicroInterpreter static_interpreter(model, op_resolver, (uint8_t*)tensor_arena, tensor_arena_size, &micro_error_reporter);

	if(static_interpreter.AllocateTensors()!= kTfLiteOk) {
		return false;
	}
	int_ptr = &static_interpreter;
	input = static_interpreter.input(0);
	output = static_interpreter.output(0);

	/* Diagnostic: is the output tensor's quantization per-tensor (one shared
	 * scale/zero_point) or per-channel (one per column of the [N,8] output -
	 * x,y,w,h,obj,class0,class1,class2)? decode_boxes() currently assumes
	 * per-tensor (scale->data[0]/zero_point->data[0] applied to every
	 * column) - if size here is 8, not 1, that assumption is wrong and each
	 * column needs its own scale[c]/zero_point[c]. Print once at boot,
	 * cheap, not a hot-path cost like the per-row bbox dump. */
	{
		TfLiteAffineQuantization* q = (TfLiteAffineQuantization*)(output->quantization.params);
		xprintf("[quant] output scale->size=%d zero_point->size=%d\n",
			(int)q->scale->size, (int)q->zero_point->size);
		for (int i = 0; i < q->scale->size; ++i) {
			/* %f is unusable here - this app's newlib-nano isn't linked with
			 * -u _printf_float (confirmed: no other xprintf in this codebase
			 * uses %f), so print scale as a scaled integer instead. */
			xprintf("[quant]   [%d] scale_x1e6=%ld zero_point=%d\n",
				i, (long)(q->scale->data[i] * 1000000.0f), (int)q->zero_point->data[i]);
		}
	}

	xprintf("initial done\n");

	return ercode;
}

namespace {

/* One decoded/NMS-ready candidate box, in MODEL_INPUT_W/H pixel space,
 * TOP-LEFT-CORNER-based (x,y = box corner, not center) - see decode_boxes()'s
 * comment for why; this is the opposite of what earlier reverse-engineering
 * assumed. */
struct yolo_box {
    float x, y, w, h;
    float score;
    int class_id;
};

/* 2026-07-10: ground-truthed against Seeed's own reference decoder,
 * SSCMA-Micro's YoloV5::generalPostProcess() (/home/orangepi/SSCMA-Micro/
 * sscma/core/model/ma_model_yolov5.cpp) - Swift-YOLO is a YOLOv5 derivative
 * and shares its post-processing. Default there is ma_model_detector.cpp's
 * Detector() ctor: threshold_nms_(0.45). The earlier 0.1 was tuned against
 * data corrupted by the center/corner bug (see decode_boxes()) - overlap
 * looked pathological because boxes for the same hand were shifted by
 * different amounts (proportional to each one's own w/2,h/2), not because
 * real overlap was ever that high. Reverted to the reference's 0.45 now that
 * the actual coordinates are correct.
 *
 * 2026-07-10: now runtime-adjustable via AT+TIOU (app_get_tiou(), default 45
 * - see sscma_cam_mic.c) instead of a compile-time constant, so this and
 * AT+TSCORE can both be tuned live without reflashing. */
#define MAX_YOLO_DETECTIONS 10

/* Corner-based (x,y = top-left, matches ma_nms.h's compute_iou()) - NOT
 * center-based like the pre-2026-07-10 version of this function. */
static float box_iou(const yolo_box &a, const yolo_box &b)
{
    float ax1 = a.x, ay1 = a.y;
    float ax2 = a.x + a.w, ay2 = a.y + a.h;
    float bx1 = b.x, by1 = b.y;
    float bx2 = b.x + b.w, by2 = b.y + b.h;

    float ix1 = std::max(ax1, bx1), iy1 = std::max(ay1, by1);
    float ix2 = std::min(ax2, bx2), iy2 = std::min(ay2, by2);
    float iw = ix2 - ix1, ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;

    float inter = iw * ih;
    float uni = a.w * a.h + b.w * b.h - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

/* 2026-07-10: plain IoU (above) misses the case where a small box sits
 * almost entirely inside a much bigger one - the bigger box's extra area
 * inflates the union, so IoU stays well under NMS_IOU_THRESHOLD even at
 * 100% containment (hardware-confirmed: run_20260710_015017 log had pairs
 * with containment=1.000 - the smaller box's corners were fully inside the
 * larger one - yet IoU only 0.28-0.44, so plain-IoU NMS kept both as
 * separate detections of what was clearly one hand). This is a known
 * weakness of greedy IoU-NMS in general - SSCMA-Micro's own reference
 * (ma_nms.cpp) has the identical gap, so this check is a deliberate,
 * app-specific addition on top of the reference, not a spec deviation
 * anyone will trip over expecting official parity. Returns intersection
 * area as a fraction of the SMALLER box's own area. */
static float box_containment(const yolo_box &a, const yolo_box &b)
{
    float ax1 = a.x, ay1 = a.y, ax2 = a.x + a.w, ay2 = a.y + a.h;
    float bx1 = b.x, by1 = b.y, bx2 = b.x + b.w, by2 = b.y + b.h;

    float ix1 = std::max(ax1, bx1), iy1 = std::max(ay1, by1);
    float ix2 = std::min(ax2, bx2), iy2 = std::min(ay2, by2);
    float iw = ix2 - ix1, ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;

    float inter = iw * ih;
    float smaller = std::min(a.w * a.h, b.w * b.h);
    return smaller > 0.0f ? inter / smaller : 0.0f;
}
/* 0.9, then 0.75, both still left real hardware-observed duplicate pairs
 * unsuppressed (2026-07-10 - two rounds of hardware testing each found a
 * fresh cluster sitting just under whatever threshold was current). Per
 * user's direction: stop chasing a cutoff and suppress on ANY containment -
 * i.e. any nonzero geometric overlap between two boxes at all means one is
 * sitting (at least partly) inside the other, and for this app's single-
 * hand-in-frame use case that's always the same hand, never two legitimately
 * separate objects that happen to touch. threshold_iou above is effectively
 * superseded by this (any overlap already implies containment > 0), kept
 * only because it's what SSCMA-Micro's own reference NMS uses and there's no
 * reason to diverge from a value that was never the problem. */
#define NMS_CONTAINMENT_THRESHOLD 0.0f

/* Decode the model's raw [num_boxes, 8] output tensor into candidate boxes
 * above score_threshold. Columns = [x, y, w, h, objectness, class0, class1,
 * class2], 3 classes = paper/rock/scissors (hardware-confirmed 2026-07-10).
 *
 * 2026-07-10 rewrite: ground-truthed against SSCMA-Micro's own reference
 * decoder for this exact model family, YoloV5::generalPostProcess() (see
 * /home/orangepi/SSCMA-Micro/sscma/core/model/ma_model_yolov5.cpp - Swift-
 * YOLO is a YOLOv5 derivative, confirmed by the user). Three things the
 * earlier from-first-principles reverse-engineering (SSCMA model card + on-
 * hardware raw-value inspection) got wrong, all now fixed to match the
 * reference bit-for-bit:
 *   1. NO sigmoid anywhere. The earlier assumption that objectness/class
 *      were raw pre-sigmoid logits was wrong - the export already bakes
 *      whatever activation it needs in, so the tensor's dequantized value
 *      *is* the score (optionally /100 if the tensor's own scale implies a
 *      0..100 range rather than 0..1 - see "normalized" below).
 *   2. Final score is objectness ALONE, not objectness*best_class_score.
 *      Class is only used to pick the label. And picking it doesn't even
 *      need dequantizing: argmax over the raw int8 class bytes gives the
 *      same answer as argmax over the dequantized floats (dequant is a
 *      monotonic affine map, same scale/zero_point for every class column),
 *      so the reference just compares raw bytes directly.
 *   3. x,y are already the TOP-LEFT CORNER, not the box center - do not
 *      subtract w/2,h/2. ma_nms.h's compute_iou() treats box.x/box.y as the
 *      left/top edge with no center adjustment anywhere in the int8 code
 *      path, and generalPostProcess() never applies one either. The earlier
 *      "center-based" assumption was never verified against a reference,
 *      just carried over from how tflm_yolov8_od's own (different) model
 *      happens to encode its output - it silently shifted every reported
 *      box up-left by its own half-width/half-height, which is also why
 *      overlap between boxes for the same physical hand looked so
 *      inconsistent (see NMS_IOU_THRESHOLD's comment above). */
/* Hard cap on retained pre-NMS candidates. Fixed-size, stack/static-only -
 * no heap allocation anywhere in this per-frame hot path. This app runs
 * with very little SRAM slack (~60KB free once static buffers/tensor arena
 * are accounted for - see TENSOR_ARENA_BUFSIZE's comment); an earlier
 * std::vector-based version of this code crashed the firmware outright
 * with std::bad_alloc once enough of the num_boxes=2268 candidates passed
 * threshold at once, and even after capping the vector's *size* the
 * repeated heap churn (allocate/free every single frame, forever) still
 * intermittently stalled the camera event loop after a handful of frames
 * (heap fragmentation on a newlib-nano allocator this tight) - see the
 * 2026-07-08 session notes. run_nms() only ever keeps the top
 * MAX_YOLO_DETECTIONS anyway, so retaining far more than that pre-NMS buys
 * nothing. */
constexpr int kMaxCandidates = 128;

int decode_boxes(TfLiteTensor* out, float score_threshold, yolo_box* boxes_out)
{
    int dims = out->dims->size;
    int num_boxes = out->dims->data[dims - 2];
    int row_width = out->dims->data[dims - 1];
    int num_classes = row_width - 5;

    /* Per-tensor vs per-channel quantization: if the output's scale/
     * zero_point array has only 1 entry, every column (x,y,w,h,obj,class0..)
     * shares it (what this code originally assumed). If it has row_width
     * entries instead, each column has its own scale[c]/zero_point[c] and
     * indexing everything with [0] would silently apply x's quant params to
     * objectness/class - see cv_init()'s "[quant]" boot print for which case
     * this model actually is. dequant() below is written to be correct
     * either way. */
    TfLiteAffineQuantization* quant = (TfLiteAffineQuantization*)(out->quantization.params);
    bool per_channel = quant->scale->size > 1;
    auto dequant = [&](int col, int8_t q) -> float {
        int idx = per_channel ? col : 0;
        return ((float)q - quant->zero_point->data[idx]) * quant->scale->data[idx];
    };
    /* ma_model_yolov5.cpp: "bool normalized = scale < 0.1f;" - if the
     * objectness column's scale is small, the dequantized value already
     * lands in ~[0,1] (a true probability); if not, it's ~[0,100] and needs
     * /100 to match score_threshold's [0,1] scale. Reference always reads
     * this off the (single, per-tensor) scale - mirrored here off column 4's
     * in case this model turns out to be per-channel after all. */
    bool normalized = quant->scale->data[per_channel ? 4 : 0] < 0.1f;

    /* Always scan every candidate and keep the kMaxCandidates HIGHEST-scoring
     * ones that pass threshold, evicting the current worst kept candidate
     * when a better one shows up past the cap. The previous version capped
     * by *acceptance count* ("stop once we've taken 128"), which silently
     * dropped the real, high-confidence detection whenever >128 low/
     * borderline-score candidates happened to sit earlier in tensor order
     * (confirmed on hardware: a real gesture's clear top candidate at
     * index ~1809 never made it into a single reported box, because ~128
     * marginal-score noise candidates at indices 0-127 filled the array
     * first every time - see the 2026-07-08 session notes). */
    int n = 0;
    int min_idx_in_out = -1;
    float min_score_in_out = 0.0f;

    for (int i = 0; i < num_boxes; ++i) {
        const int8_t* row = out->data.int8 + i * row_width;

        float score = dequant(4, row[4]);
        score = normalized ? score : score / 100.0f;
        if (score < score_threshold) continue;
        if (n >= kMaxCandidates && score <= min_score_in_out) continue;

        /* argmax over raw int8 bytes directly - dequant is a monotonic
         * affine map (same scale/zero_point across the class columns), so
         * this picks the same class as argmax-over-dequantized-floats would,
         * without needing to actually dequantize. */
        int8_t max_class_raw = -128;
        int best_class = 0;
        for (int c = 0; c < num_classes; ++c) {
            if (row[5 + c] > max_class_raw) { max_class_raw = row[5 + c]; best_class = c; }
        }

        yolo_box b;
        b.x = dequant(0, row[0]);   /* top-left corner, model-pixel space */
        b.y = dequant(1, row[1]);   /* top-left corner, model-pixel space */
        b.w = dequant(2, row[2]);
        b.h = dequant(3, row[3]);
        b.score = score;
        b.class_id = best_class;

        int dst;
        if (n < kMaxCandidates) {
            dst = n++;
        } else {
            dst = min_idx_in_out;  /* evict the current worst kept candidate */
        }
        boxes_out[dst] = b;

        if (n >= kMaxCandidates) {
            /* recompute the new minimum among all kept candidates */
            min_idx_in_out = 0;
            min_score_in_out = boxes_out[0].score;
            for (int k = 1; k < n; ++k) {
                if (boxes_out[k].score < min_score_in_out) {
                    min_score_in_out = boxes_out[k].score;
                    min_idx_in_out = k;
                }
            }
        }
    }
    return n;
}

/* Greedy class-agnostic NMS over already-thresholded candidates, sorted by
 * descending score, operating entirely over fixed-size stack arrays (see
 * kMaxCandidates' comment above for why - no std::vector/heap allocation
 * here). Writes up to MAX_YOLO_DETECTIONS indices into keep_out, returns how
 * many.
 *
 * Deliberately class-agnostic (suppresses across classes too), NOT the
 * per-class NMS tflm_yolov8_od's yolov8_NMSBoxes() uses. Per-class NMS is
 * right for COCO-style multi-object scenes where two genuinely different
 * objects can legitimately overlap. This model's 3 gesture classes look
 * visually similar to each other (rock/scissors especially), so neighboring
 * anchors over the *same* physical hand frequently disagree on class -
 * confirmed on hardware: a single held-up gesture was reported as several
 * overlapping boxes split across two different class_ids, because per-class
 * NMS only suppresses boxes sharing the same predicted class. For a
 * single-hand-in-frame use case, collapsing every high-IoU cluster to its
 * single best-scoring box (regardless of what class each member guessed) is
 * the correct behavior - see the 2026-07-08 session notes. */
int run_nms(const yolo_box* boxes, int n, int* keep_out)
{
    int order[kMaxCandidates];
    bool removed[kMaxCandidates] = {};
    for (int i = 0; i < n; ++i) order[i] = i;
    std::sort(order, order + n, [&](int a, int b) { return boxes[a].score > boxes[b].score; });

    int n_keep = 0;
    for (int oi = 0; oi < n && n_keep < MAX_YOLO_DETECTIONS; ++oi) {
        int i = order[oi];
        if (removed[i]) continue;
        keep_out[n_keep++] = i;
        for (int oj = oi + 1; oj < n; ++oj) {
            int j = order[oj];
            if (removed[j]) continue;
            /* box_containment() (see its comment) catches the case plain IoU
             * misses: a much-smaller box sitting almost entirely inside the
             * kept one. */
            if (box_iou(boxes[i], boxes[j]) > (float)app_get_tiou() / 100.0f ||
                box_containment(boxes[i], boxes[j]) > NMS_CONTAINMENT_THRESHOLD) {
                removed[j] = true;
            }
        }
    }
    return n_keep;
}

int run_yolo_detect(std::forward_list<el_box_t> &out_boxes)
{
    if (int_ptr == nullptr) {
        /* cv_init() failed to load the model (see sscma_cam_mic.c) - no
         * inference possible, but the caller (cam_handle_frame) still
         * needs a clean "no detections" result rather than a null deref. */
        return -1;
    }

    uint32_t img_w = app_get_raw_width();
    uint32_t img_h = app_get_raw_height();
    uint32_t raw_addr = app_get_raw_addr();

    /* Resize via the Himax helium lib, same as tflm_yolov8_od: planar BGR in
     * ([[BBB..],[GGG..],[RRR..]] - the HW5x5 RGB-demosaic WDMA3 layout),
     * interleaved rgb out, output width must be a multiple of 4 (192 is) -
     * see img_proc_helium.h's doc comment. MVE-accelerated, which is why
     * configENABLE_FPU=1 (sscma_cam_mic.mk) is load-bearing: without
     * S16-S31/Q4-Q7 saved across context switches, audio_task preempting
     * mid-resize corrupted the vectorized loop's address registers and
     * wedged the bus (the former "AI stream stalls after a few frames"
     * hang - it was never this function's fault; CMSIS-CV's equivalent,
     * also MVE via -O2 auto-vectorization, stalled identically). */
    float w_scale = (float)(img_w - 1) / (MODEL_INPUT_W - 1);
    float h_scale = (float)(img_h - 1) / (MODEL_INPUT_H - 1);
    hx_lib_image_resize_BGR8U3C_to_RGB24_helium((uint8_t*)raw_addr,
            (uint8_t*)input->data.data,
            (int)img_w, (int)img_h, MODEL_INPUT_C,
            MODEL_INPUT_W, MODEL_INPUT_H, w_scale, h_scale);

    for (size_t i = 0; i < input->bytes; ++i) {
        *((int8_t *)input->data.data + i) = *((int8_t *)input->data.data + i) - 128;
    }

    TfLiteStatus invoke_status = int_ptr->Invoke();
    if (invoke_status != kTfLiteOk) {
        xprintf("yolo detect invoke fail\n");
        return -1;
    }

    float score_threshold = (float)app_get_tscore() / 100.0f;
    static yolo_box boxes[kMaxCandidates];  /* .bss, not stack/heap - see kMaxCandidates' comment */
    int n_boxes = decode_boxes(output, score_threshold, boxes);
    int keep[MAX_YOLO_DETECTIONS];
    int n_keep = run_nms(boxes, n_boxes, keep);

    float scale_x = (float)img_w / (float)MODEL_INPUT_W;
    float scale_y = (float)img_h / (float)MODEL_INPUT_H;

    for (int ki = 0; ki < n_keep; ++ki) {
        const yolo_box &b = boxes[keep[ki]];
        /* b.x/b.y are already the top-left corner (see decode_boxes()) - no
         * center-to-corner subtraction here, unlike the pre-2026-07-10
         * version of this function. */
        float left = b.x * scale_x;
        float top  = b.y * scale_y;
        if (left < 0.0f) left = 0.0f;
        if (top < 0.0f) top = 0.0f;

        el_box_t box{};
        box.x      = (uint16_t)left;
        box.y      = (uint16_t)top;
        box.w      = (uint16_t)(b.w * scale_x);
        box.h      = (uint16_t)(b.h * scale_y);
        box.score  = (uint8_t)std::min(100.0f, b.score * 100.0f);
        box.target = (uint8_t)b.class_id;
        out_boxes.push_front(box);
    }

    return 0;
}

} // namespace

void cam_handle_frame(void)
{
    cam_stream_mode_e mode = app_get_cam_stream_mode();
    if (mode == CAM_STREAM_IDLE) {
        return;
    }

    uint32_t jpeg_sz = 0, jpeg_addr = 0;
    cisdp_get_jpginfo(&jpeg_sz, &jpeg_addr);
    hx_InvalidateDCache_by_Addr((volatile void *)jpeg_addr, sizeof(uint8_t) * jpeg_sz);

    el_img_t img{};
    img.data   = (uint8_t *)jpeg_addr;
    img.size   = jpeg_sz;
    img.width  = app_get_raw_width();
    img.height = app_get_raw_height();
    img.format = EL_PIXEL_FORMAT_JPEG;
    img.rotate = EL_PIXEL_ROTATE_0;

    if (mode == CAM_STREAM_SAMPLE) {
        event_reply_named_with_payload("SAMPLE", "", "image", img.data, img.size);
        return;
    }

    /* CAM_STREAM_INVOKE */
    std::forward_list<el_box_t> boxes;
    if (run_yolo_detect(boxes) != 0) {
        return;
    }

    std::string prefix_fields = concat_strings(", ", box_results_2_json_str(boxes));

    if (app_get_result_only()) {
        event_reply_named("INVOKE", prefix_fields);
    } else {
        /* Streamed instead of img_2_json_str()+concat_strings(): a ~15-20KB
         * base64 JPEG materialized as one heap string (here, in
         * event_reply_named's by-value copy, and again inside its own
         * concat_strings) was exhausting the heap under simultaneous
         * camera+audio load - see event_reply_named_with_payload()'s
         * comment. */
        event_reply_named_with_payload("INVOKE", prefix_fields, "image", img.data, img.size);
    }
}

int cv_deinit()
{
	//TODO: add more deinit items here if need.
	return 0;
}
