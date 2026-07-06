/*
 * cvapp.cpp
 *
 * Ethos-U55 + TFLite-Micro person-detect inference, copied from
 * allon_sensor_tflm. Unlike that app, cam_handle_frame() also owns turning
 * the result into an SSCMA-style JSON/base64 reply (see send_result.h),
 * gated by the stream mode at_cmd.cpp last set - that keeps all the C++
 * (std::string/forward_list) usage inside this translation unit, out of
 * the plain-C frame-ready event handler in sscma_cam_mic.c.
 */

#include <cstdio>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
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

#include "person_detect_model_data_vela.h"
#include "common_config.h"
#include "send_result.h"
#include "sscma_cam_mic.h"

#define LOCAL_FRAQ_BITS (8)
#define SC(A, B) ((A<<8)/B)

#define INPUT_SIZE_X 96
#define INPUT_SIZE_Y 96

#ifdef TRUSTZONE_SEC
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE	BASE_ADDR_APB_U55_CTRL
#endif
#endif

#define TENSOR_ARENA_BUFSIZE  (125*1024)
__attribute__(( section(".bss.NoInit"))) uint8_t tensor_arena_buf[TENSOR_ARENA_BUFSIZE] __ALIGNED(32);

using namespace std;

namespace {

constexpr int tensor_arena_size = TENSOR_ARENA_BUFSIZE;
uint32_t tensor_arena = (uint32_t)tensor_arena_buf;

struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */
tflite::MicroInterpreter *int_ptr=nullptr;
TfLiteTensor* input, *output;
};


static void img_rescale(
        const uint8_t*in_image,
        const int32_t width,
        const int32_t height,
        const int32_t nwidth,
        const int32_t nheight,
        int8_t*out_image,
        const int32_t nxfactor,
        const int32_t nyfactor) {
    int32_t x,y;
    int32_t ceil_x, ceil_y, floor_x, floor_y;

    int32_t fraction_x,fraction_y,one_min_x,one_min_y;
    int32_t pix[4];//4 pixels for the bilinear interpolation
    int32_t out_image_fix;

    for (y = 0; y < nheight; y++) {//compute new pixels
        for (x = 0; x < nwidth; x++) {
            floor_x = (x*nxfactor) >> LOCAL_FRAQ_BITS;//left pixels of the window
            floor_y = (y*nyfactor) >> LOCAL_FRAQ_BITS;//upper pixels of the window

            ceil_x = floor_x+1;//right pixels of the window
            if (ceil_x >= width) ceil_x=floor_x;//stay in image

            ceil_y = floor_y+1;//bottom pixels of the window
            if (ceil_y >= height) ceil_y=floor_y;

            fraction_x = x*nxfactor-(floor_x << LOCAL_FRAQ_BITS);//strength coefficients
            fraction_y = y*nyfactor-(floor_y << LOCAL_FRAQ_BITS);

            one_min_x = (1 << LOCAL_FRAQ_BITS)-fraction_x;
            one_min_y = (1 << LOCAL_FRAQ_BITS)-fraction_y;

            pix[0] = in_image[floor_y * width + floor_x];//store window
            pix[1] = in_image[floor_y * width + ceil_x];
            pix[2] = in_image[ceil_y * width + floor_x];
            pix[3] = in_image[ceil_y * width + ceil_x];

            //interpolate new pixel and truncate it's integer part
            out_image_fix = one_min_y*(one_min_x*pix[0]+fraction_x*pix[1])+fraction_y*(one_min_x*pix[2]+fraction_x*pix[3]);
            out_image_fix = out_image_fix >> (LOCAL_FRAQ_BITS * 2);
            out_image[nwidth*y+x] = out_image_fix-128;
        }
    }
}

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

	static const tflite::Model*model = tflite::GetModel((const void *)g_person_detect_model_data_vela);

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
	static tflite::MicroMutableOpResolver<1> op_resolver;

	if (kTfLiteOk != op_resolver.AddEthosU()){
		xprintf("Failed to add Arm NPU support to op resolver.");
		return false;
	}

	static tflite::MicroInterpreter static_interpreter(model, op_resolver, (uint8_t*)tensor_arena, tensor_arena_size, &micro_error_reporter);

	if(static_interpreter.AllocateTensors()!= kTfLiteOk) {
		return false;
	}
	int_ptr = &static_interpreter;
	input = static_interpreter.input(0);
	output = static_interpreter.output(0);

	xprintf("initial done\n");

	return ercode;
}

namespace {

struct person_detect_result_t {
    int8_t person_score;
    int8_t no_person_score;
};

int run_person_detect(person_detect_result_t *out)
{
	//give image to input tensor
	img_rescale((uint8_t*)app_get_raw_addr(), app_get_raw_width(), app_get_raw_height(), INPUT_SIZE_X, INPUT_SIZE_Y,
			input->data.int8, SC(app_get_raw_width(), INPUT_SIZE_X), SC(app_get_raw_height(), INPUT_SIZE_Y));

	TfLiteStatus invoke_status = int_ptr->Invoke();

	if(invoke_status != kTfLiteOk)
	{
		xprintf("invoke fail\n");
		return -1;
	}

	out->person_score = output->data.int8[1];
	out->no_person_score = output->data.int8[0];

	return 0;
}

/* Raw int8 model output has no exposed dequantization params here, so this
 * is a linear int8-range-to-percentage mapping (not a calibrated
 * probability) - good enough to threshold against AT+TSCORE. */
int score_to_percent(int8_t score)
{
    int pct = ((int)score + 128) * 100 / 255;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
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
    person_detect_result_t r;
    if (run_person_detect(&r) != 0) {
        return;
    }

    int class_id  = (r.person_score >= r.no_person_score) ? 1 : 0;
    int score_pct = (class_id == 1) ? score_to_percent(r.person_score)
                                     : (100 - score_to_percent(r.person_score));

    std::forward_list<el_class_t> classes;
    if (score_pct >= (int)app_get_tscore()) {
        el_class_t c{};
        c.score  = (uint16_t)score_pct;
        c.target = (uint16_t)class_id;
        classes.push_front(c);
    }

    std::string prefix_fields = concat_strings(", ", results_2_json_str(classes));
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
