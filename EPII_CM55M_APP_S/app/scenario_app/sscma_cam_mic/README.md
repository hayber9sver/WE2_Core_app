# SSCMA Camera + Microphone (AT-command streaming)

`sscma_cam_mic` runs the camera (JPEG capture + NPU inference) and the PDM
microphone as two independent FreeRTOS tasks (`cam_task` / `audio_task`),
both driven and streamed out over the same AT-command UART link. Camera
frames/results are sent as JSON with base64-encoded image data; audio is
sent as a dedicated binary frame (see `send_audio_binary_frame()` in
`send_result.cpp`) to keep its wire size down.

The camera runs a YOLO-style object-detect model (192x192x3 RGB in,
`[N,8]`=`[x,y,w,h,score,class_id,reserved,reserved]` out), flash-loaded
(`FLASH_XIP_MODEL=1`, see `common_config.h`) rather than compiled into the
firmware - it's flashed as a separate xmodem step (below), independently of
the firmware image.

## Building and Flashing

1. Set `APP_TYPE` in [`EPII_CM55M_APP_S/makefile`](../../../makefile):
    ```makefile
    APP_TYPE = sscma_cam_mic
    ```
2. Build the firmware (see the top-level repo README for the full Linux
   build walkthrough). `make` only produces the ELF at
   `obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf`;
   copy it over `we2_image_gen_local/input_case1_secboot/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf`
   before running `we2_local_image_gen` to produce a flashable image from
   the build you actually just made.
3. Flash `output.img` **and** the `.tflite` model in the same xmodem
   command - the model's burn-position argument is the *bare flash offset*
   (no `0x3A` prefix; that prefix is only for the CPU-side XIP-mapped
   address `common_config.h`'s `MODEL_FLASH_ADDR` uses, see that file's
   comment for the full explanation):
    ```
    python3 xmodem/xmodem_send.py --port=/dev/ttyACM0 --baudrate=921600 \
        --protocol=xmodem --file=we2_image_gen_local/output_case1_sec_wlcsp/output.img \
        --model="<path-to-your-model>.tflite 0xB7B000 0x00000"
    ```
   **Do not use `0x180000`** even though an older comment in
   `allon_sensor_tflm/common_config.h` mentions it - on real hardware the
   off-line-burn bootloader completed the full transfer (0 xmodem errors)
   but then rejected the *address itself* (3x CAN + a truncated
   `Xmodem: ERR_` message, confirmed via a raw-byte capture). `0xB7B000` is
   the address every other model-flashing app in this repo actually uses
   and has confirmed working (`tflm_yolov8_od`, `kws_pdm_record`).
   Sizing check before flashing a new/different model: the model's own size
   must not run past the flash chip's end (16MB) - see the sizing comment
   above `MODEL_FLASH_ADDR` in `common_config.h` for the worked example and
   the exact numbers as of the model currently in use.
4. Press `reset` on the board.
5. `tools/capture_cam_mic.py` is a Python reference client: drives the
   AT+SENSOR/AT+INVOKE/AT+ASR/AT+ASAMPLE commands and saves received
   frames/audio to disk. `tools/run_test_matrix.sh` runs it across the full
   camera-resolution x preview x audio-rate combination matrix.

## Known limitation: UART link bandwidth ceiling at 32kHz audio + 640x480

The AT-command link runs at 921600 baud, i.e. **~92160 bytes/sec** of
effective payload throughput (8N1 framing). Audio and camera share this one
link, and at the two extremes together the *combined demanded* bitrate
exceeds that ceiling:

- **32kHz mono 16-bit PCM alone needs ~64000 bytes/sec** (32000 samples/s x
  2 bytes), even after switching ASAMPLE to raw binary framing (the older
  base64+JSON encoding needed ~85000+ bytes/sec just for this, which is why
  32kHz audio was unusable at all before that fix - see the audio-click
  investigation notes). That alone is already ~69% of the link's total
  capacity.
- That leaves only **~28000 bytes/sec (~27 KB/s)** of headroom for the
  camera. A 640x480 JPEG frame from this sensor/quality setting is
  typically **~15-20 KB** once base64+JSON encoded (measured ~20-21 KB on
  the wire in testing) - so the link can sustain barely more than **~1
  frame/second** of 640x480 preview concurrently with 32kHz audio before
  demand outpaces the wire.
- Symptom when the camera (or its JPEG encode timing) tries to push frames
  faster than that: the shared UART falls behind the producers on both
  sides, observed as either a dropped/corrupted image frame or a
  CRC-failed, discarded audio chunk (the PDM ring's writer catching up to
  the reader - see `pdm_audio_debug_log()` history in the audio-click
  memory for the underlying producer/consumer mechanics). In practice this
  has only ever manifested as a single glitch right at the start of a
  capture in testing (see the 18-combo test matrix results), but it is a
  **real, physical bandwidth ceiling of the single 921600-baud link**, not
  a bug that more buffering alone can fix - the total data volume genuinely
  does not fit.
- 16kHz audio (~32000 bytes/sec) leaves ~60000 bytes/sec (~58 KB/s) for the
  camera instead - roughly 3 frames/sec of 640x480, or comfortably more at
  320x240/160x112 - so 16kHz + camera is well inside the link's budget and
  has not shown this symptom.

**This is the current hard limit of running audio + full-resolution camera
preview simultaneously over one AT UART link.** Options if more headroom is
ever needed: raise the UART baud rate, lower the audio rate, lower the
camera resolution/frame rate/JPEG quality, or move one of the two streams
(e.g. audio) onto a second physical UART - none of these have been
implemented, this section only documents the constraint as currently
understood.
