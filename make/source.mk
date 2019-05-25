COMMON_SRC = \
            build/build_config.c \
            build/debug.c \
            build/version.c \
            $(TARGET_DIR_SRC) \
            main.c \
            $(addprefix pg/,$(notdir $(wildcard $(SRC_DIR)/pg/*.c))) \
            $(addprefix common/,$(notdir $(wildcard $(SRC_DIR)/common/*.c))) \
            $(addprefix config/,$(notdir $(wildcard $(SRC_DIR)/config/*.c))) \
            drivers/serial.c \
            drivers/serial_uart.c \
            fc/runtime_config.c \
            io/beeper.c \
			io/serial.c \
            msp/msp.c \
            msp/msp_box.c \
            msp/msp_serial.c \
            drivers/accgyro/acc_bma280.c \
			drivers/accgyro/gyro_bmg160.c \
			drivers/accgyro/gyro_sync.c \
            drivers/accgyro/gyro_sync.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/gyro.c \
            sensors/initialisation.c \
            flight/imu.c \
			fc/rc_modes.c \

COMMON_DEVICE_SRC = \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

COMMON_SRC := $(COMMON_SRC) $(COMMON_DEVICE_SRC)

ifeq ($(SIMULATOR_BUILD),yes)
TARGET_FLAGS := -DSIMULATOR_BUILD $(TARGET_FLAGS)
endif

SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

ifneq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \

SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) 

# F4 and F7 optimizations
ifneq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) 
endif #!F3
endif #!F1

# check if target.mk supplied
SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC) $(VARIANT_SRC)

# Files that should not be optimized, useful for debugging IMPRECISE cpu faults.
# Specify FULL PATH, e.g. "./lib/main/STM32F7/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c"
NOT_OPTIMISED_SRC := $(NOT_OPTIMISED_SRC) \

ifneq ($(DSP_LIB),)

INCLUDE_DIRS += $(DSP_LIB)/Include

SRC += $(DSP_LIB)/Source/BasicMathFunctions/arm_mult_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_rfft_fast_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_cfft_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_rfft_fast_init_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_cfft_radix8_f32.c
SRC += $(DSP_LIB)/Source/CommonTables/arm_common_tables.c

SRC += $(DSP_LIB)/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c
SRC += $(DSP_LIB)/Source/StatisticsFunctions/arm_max_f32.c

SRC += $(wildcard $(DSP_LIB)/Source/*/*.S)
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
SRC += \
            drivers/flash.c \
            drivers/flash_m25p16.c \
            drivers/flash_w25m.c \
            io/flashfs.c \
            pg/flash.c \
            $(MSC_SRC)
endif

SRC += $(COMMON_SRC)

#excludes
SRC   := $(filter-out $(MCU_EXCLUDES), $(SRC))

ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
SRC += \
            drivers/sdcard.c \
            drivers/sdcard_spi.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c \
            $(MSC_SRC)
endif

ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
SRC += \
            drivers/sdcard.c \
            drivers/sdcard_sdio_baremetal.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c \
            pg/sdio.c \
            $(MSC_SRC)
endif

ifneq ($(filter VCP,$(FEATURES)),)
SRC += $(VCP_SRC)
endif

ifneq ($(filter MSC,$(FEATURES)),)
SRC += $(MSC_SRC)
endif
# end target specific make file checks

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src
