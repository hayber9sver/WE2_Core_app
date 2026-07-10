/*
 * common_config.h
 *
 *  Created on: Nov 22, 2022
 *      Author: bigcat-himax
 */

#ifndef APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_
#define APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_

/** MODEL location:
 *	0: model file is a c file which will locate to memory.
 *		in this example, model data is "person_detect_model_data_vela.cc" file.
 *
 *	1: model file will off-line burn to dedicated location in flash,
 *		use flash memory mapped address to load model.
 *		in this example, model data is pre-burn to flash address: 0x180000
 * **/
#define FLASH_XIP_MODEL 1
#define MEM_FREE_POS		(BOOT2NDLOADER_BASE) ////0x3401F000

/* YOLO-style 192x192x3 RGB object-detect model, flash-loaded (see README).
 *
 * MODEL_FLASH_ADDR (below) is the CPU-side XIP-mapped address tflite::
 * GetModel() reads from - it always has the 0x3A prefix (flash aliased at
 * 0x3A000000).
 *
 * xmodem's --model= burn-position argument is the BARE OFFSET instead (no
 * 0x3A prefix):
 *   xmodem --model="<path-to-tflite> 0xB7B000 0x00000"
 *
 * NOT 0x180000: allon_sensor_tflm's own comment mentions 0x180000 as a
 * valid FLASH_XIP_MODEL=1 address, but nothing in this repo actually
 * demonstrates burning there successfully, and on real hardware the
 * off-line-burn bootloader rejected it (xmodem transfer of the full model
 * completed with 0 errors, but the receiver then sent 3x CAN and printed
 * "Xmodem: ERR_" - i.e. its own post-transfer validation rejected the
 * *address*, not the transfer). 0xB7B000 is what every other model-flashing
 * app in this repo actually uses and has confirmed working
 * (tflm_yolov8_od, kws_pdm_record; tflm_fd_fm's 3 models start even higher
 * at 0x200000+) - matching that convention instead.
 *
 * Sizing: as of writing, this model is 824016 bytes (0xC92D0), occupying
 * flash offset [0xB7B000, 0xC4E2D0) - nowhere near the 16MB flash chip's
 * end, and far past this app's own flash footprint (~141932 bytes/0x22A6C).
 * If a future model swap changes size materially, re-check this math. */
#define MODEL_FLASH_ADDR 0x3AB7B000

/* Matches the "Sizing" note above - used only to fill in AT+INVOKE's
 * "model":{"size":...} field (see at_cmd.cpp). Not read back from flash at
 * runtime; re-check this if the flashed model ever changes materially. */
#define MODEL_FLASH_SIZE 824016

#endif /* APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_ */
