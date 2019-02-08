PRJ_PATH = .
DEBUG_SCRIPT_FLASH = $(PRJ_PATH)/flash.gdb
ARCH = cortex-m0plus
PART = samd20j18
TARGET_FLASH = test_flash.elf
BUILD_DIR = build


# List of C source files.
CSRCS = \
	src/Sensors/bma2x2_support.c \
	src/Sensors/bma2x2.c \
	src/Sensors/bmg160_support.c \
	src/Sensors/bmg160.c \
	src/Sensors/bmm050_support.c \
	src/Sensors/bmm050.c \
	src/ASF_Support/clock_support.c \
	src/ASF_Support/spi_support.c \
    src/ASF_Support/i2c_support.c \
	src/ASF_Support/tc_support.c \
	src/ASF_Support/usart_support.c \
	src/bmf055.c \
	src/MahonyAHRS.c \
	src/main.c

CSRCS += $(ASF_CSRCS)
	
# List of assembler source files.
ASSRCS = \

# List of include paths.
INC_PATH = \
	src \
	src/ASF_Support \
	src/Sensors \
	src/config

INC_PATH += $(ASF_INC_PATH)    

# Additional search paths for libraries.
LIB_PATH = src/ASF/thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  arm_cortexM0l_math                                

       
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
       -D TC_ASYNC=true

# Extra flags to use when linking
LDFLAGS = -u _printf_float


# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 
