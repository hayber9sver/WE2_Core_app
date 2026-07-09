override SCENARIO_APP_SUPPORT_LIST := $(APP_TYPE)

APPL_DEFINES += -DSSCMA_CAM_MIC
APPL_DEFINES += -DIP_xdma
APPL_DEFINES += -DEVT_DATAPATH

# os/freertos's FreeRTOSConfig.h defaults configENABLE_FPU to 0 (its #ifndef
# fallback - nothing in the tree ever sets it), so the CM55 port skipped
# saving S16-S31 (= MVE Q4-Q7) on context switch. This app runs two
# preempting tasks and its image-resize hot loops are auto-vectorized MVE
# (-mcpu=cortex-m55 -O2) whose gather-load offsets live in exactly those
# callee-saved Q registers: a 5ms audio_task preemption landing mid-resize
# clobbered them, and the next gather read from a garbage address wedged the
# bus - CPU frozen, no fault, no prints (hw-confirmed: heartbeat task died
# with it). Single-task reference apps (tflm_yolov8_od) never context-switch,
# which is why the identical datapath/resize code never stalled there.
APPL_DEFINES += -DconfigENABLE_FPU=1

APPL_DEFINES += -DDBG_MORE

EVENTHANDLER_SUPPORT = event_handler
EVENTHANDLER_SUPPORT_LIST += evt_datapath

##
# library support feature
# Add new library here
# The source code should be loacted in ~\library\{lib_name}\
##
LIB_SEL = pwrmgmt sensordp tflmtag2209_u55tag2205 spi_ptl spi_eeprom hxevent img_proc

##
# middleware support feature
# Add new middleware here
# The source code should be loacted in ~\middleware\{mid_name}\
##
MID_SEL =

override OS_SEL := freertos
override OS_HAL := n
override MPU := n
override TRUSTZONE := y
override TRUSTZONE_TYPE := security
override TRUSTZONE_FW_TYPE := 1
override CIS_SEL := HM_COMMON
override EPII_USECASE_SEL := drv_user_defined

CIS_SUPPORT_INAPP = cis_sensor
CIS_SUPPORT_INAPP_MODEL = cis_ov5647

ifeq ($(strip $(TOOLCHAIN)), arm)
override LINKER_SCRIPT_FILE := $(SCENARIO_APP_ROOT)/$(APP_TYPE)/sscma_cam_mic.sct
else#TOOLChain
override LINKER_SCRIPT_FILE := $(SCENARIO_APP_ROOT)/$(APP_TYPE)/sscma_cam_mic.ld
endif

##
# Add new external device here
# The source code should be located in ~\external\{device_name}\
##
#EXT_DEV_LIST +=
