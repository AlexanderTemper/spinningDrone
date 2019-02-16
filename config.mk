PRJ_PATH = .
DEBUG_SCRIPT_FLASH = $(PRJ_PATH)/flash.gdb
ARCH = cortex-m0plus
PART = samd20j18
TARGET_FLASH = test_flash.elf
BUILD_DIR = build


# List of C source files.
CSRCS = \
	src/drivers/bma2x2_support.c \
	src/drivers/bma2x2.c \
	src/drivers/bmg160_support.c \
	src/drivers/bmg160.c \
	src/drivers/bmm050_support.c \
	src/drivers/bmm050.c \
	src/sensors/sensor.c \
	src/drivers/vl53l0x.c \
	src/sensors/gyro.c \
	src/sensors/acc.c \
	src/sensors/mag.c \
	src/telemetry/simbleeBridge.c \
	src/ASF_Support/clock_support.c \
	src/ASF_Support/spi_support.c \
    src/ASF_Support/i2c_support.c \
	src/ASF_Support/tc_support.c \
	src/ASF_Support/usart_support.c \
	src/imu/imu.c \
	src/imu/MahonyAHRS.c \
	src/main.c

CSRCS += $(ASF_CSRCS)
	
# List of assembler source files.
ASSRCS = \

# List of include paths.
INC_PATH = \
	src \
	src/ASF_Support \
	src/imu \
	src/sensors \
	src/drivers \
	src/telemetry \
	src/config

INC_PATH += $(ASF_INC_PATH)    

# Additional search paths for libraries.
LIB_PATH = src/ASF/thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  arm_cortexM0l_math                                

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash 
# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
CPPFLAGS = \
       -D ARM_MATH_CM0PLUS=true \
       -D __SAMD20J18__ \
       -D BOARD=USER_BOARD \
       -D SPI_CALLBACK_MODE=true \
       -D USART_CALLBACK_MODE=true \
       -D I2C_MASTER_CALLBACK_MODE=false \
       -D TC_ASYNC=true

# Extra flags to use when linking
LDFLAGS = -u _printf_float


# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 
