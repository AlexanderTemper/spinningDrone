
cflags-gnu-y += -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
cflags-gnu-y += -Wmain -Wparentheses
cflags-gnu-y += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
cflags-gnu-y += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
cflags-gnu-y += -Wshadow -Wbad-function-cast -Wwrite-strings
cflags-gnu-y += -Wsign-compare -Waggregate-return
cflags-gnu-y += -Wmissing-declarations
cflags-gnu-y += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
cflags-gnu-y += -Wpacked -Wredundant-decls -Wnested-externs -Wlong-long
cflags-gnu-y += -Wunreachable-code
cflags-gnu-y += -Wcast-align
cflags-gnu-y += --param max-inline-insns-single=500

TARGET_FLAGS = \
    -D__FPU_PRESENT=0 \
    -DUSBD_DEBUG_LEVEL=0 \
    -DARM_MATH_CM0PLUS=true \
    -DBOARD=USER_BOARD \
    -DSPI_CALLBACK_MODE=true \
    -DUSART_CALLBACK_MODE=true \
    -DI2C_MASTER_CALLBACK_MODE=false \
    -DTC_ASYNC=true \
    $(cflags-gnu-y) \
    $(addprefix -isystem,$(ASF_INC_PATH)) \
    $(addprefix -isystem,$(SAM_LIB_PATH)/ASF_Support) \
    

TARGET_FLASH := 256

FEATURE_CUT_LEVEL = 12

HSE_VALUE :=    32000   

DEVICE_FLAGS += -DSAMD20J18

LD_SCRIPT = $(LINKER_DIR)/samd20j18_flash.ld

ARCH_FLAGS = -mcpu=cortex-m0plus -mthumb \
			 -D=__SAMD20J18__ \
			 -Wl,-larm_cortexM0l_math \
			 -Wl,-L$(ASF_PATH)/thirdparty/CMSIS/Lib/GCC \


#------------------ ASF-------------------
SAM_LIB_PATH = lib/main/SAMD20J18
ASF_PATH = $(SAM_LIB_PATH)/ASF

ASF_INC_PATH = \
	$(ASF_PATH)/common                                             \
	$(ASF_PATH)/common/utils                                       \
	$(ASF_PATH)/common/utils/interrupt                             \
	$(ASF_PATH)/common/boards                                      \
	$(ASF_PATH)/common2/boards/user_board                          \
	$(ASF_PATH)/sam0/drivers/nvm                                   \
	$(ASF_PATH)/sam0/drivers/system                                \
	$(ASF_PATH)/sam0/drivers/system/clock                          \
	$(ASF_PATH)/sam0/drivers/system/clock/clock_samd20             \
	$(ASF_PATH)/sam0/drivers/system/interrupt                      \
	$(ASF_PATH)/sam0/drivers/system/interrupt/system_interrupt_samd20 \
	$(ASF_PATH)/sam0/drivers/system/pinmux                         \
	$(ASF_PATH)/sam0/drivers/system/power                          \
	$(ASF_PATH)/sam0/drivers/system/power/power_sam_d_r_h          \
	$(ASF_PATH)/sam0/drivers/system/reset                          \
	$(ASF_PATH)/sam0/drivers/system/reset/reset_sam_d_r_h          \
	$(ASF_PATH)/sam0/drivers/port                                  \
	$(ASF_PATH)/sam0/drivers/sercom                                \
	$(ASF_PATH)/sam0/drivers/sercom/spi                            \
	$(ASF_PATH)/sam0/drivers/sercom/usart                          \
	$(ASF_PATH)/sam0/drivers/sercom/i2c                            \
	$(ASF_PATH)/sam0/drivers/sercom/i2c/i2c_samd20                 \
	$(ASF_PATH)/sam0/drivers/tc                                    \
	$(ASF_PATH)/sam0/services/eeprom/emulator/main_array           \
	$(ASF_PATH)/sam0/utils                                         \
	$(ASF_PATH)/sam0/utils/syscalls                                \
	$(ASF_PATH)/sam0/utils/syscalls/gcc                            \
	$(ASF_PATH)/sam0/utils/make                                    \
	$(ASF_PATH)/sam0/utils/cmsis/samd20                            \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include                    \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source                     \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include/component          \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include/instance           \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include/pio                \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source/gcc                 \
	$(ASF_PATH)/sam0/utils/header_files                            \
	$(ASF_PATH)/sam0/utils/preprocessor                            \
	$(ASF_PATH)/thirdparty/CMSIS/Include

	
ASF_CSRCS = \
	$(ASF_PATH)/common/utils/interrupt/interrupt_sam_nvic.c        \
	$(ASF_PATH)/common2/boards/user_board/init.c                   \
	$(ASF_PATH)/sam0/drivers/port/port.c                           \
	$(ASF_PATH)/sam0/drivers/sercom/sercom.c                       \
	$(ASF_PATH)/sam0/drivers/sercom/sercom_interrupt.c             \
	$(ASF_PATH)/sam0/drivers/sercom/spi/spi.c                      \
	$(ASF_PATH)/sam0/drivers/sercom/spi/spi_interrupt.c            \
	$(ASF_PATH)/sam0/drivers/sercom/usart/usart_interrupt.c        \
	$(ASF_PATH)/sam0/drivers/sercom/usart/usart.c                  \
	$(ASF_PATH)/sam0/drivers/sercom/i2c/i2c_samd20/i2c_master.c    \
	$(ASF_PATH)/sam0/drivers/system/clock/clock_samd20/clock.c     \
	$(ASF_PATH)/sam0/drivers/system/clock/clock_samd20/gclk.c      \
	$(ASF_PATH)/sam0/drivers/system/interrupt/system_interrupt.c   \
	$(ASF_PATH)/sam0/drivers/system/pinmux/pinmux.c                \
	$(ASF_PATH)/sam0/drivers/system/system.c                       \
	$(ASF_PATH)/sam0/drivers/tc/tc_sam_d_r_h/tc.c                  \
	$(ASF_PATH)/sam0/drivers/tc/tc_interrupt.c                     \
	$(ASF_PATH)/sam0/services/eeprom/emulator/main_array/eeprom.c  \
	$(ASF_PATH)/sam0/utils/syscalls/gcc/syscalls.c                 \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source/gcc/startup_samd20.c \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source/system_samd20.c 	
#--------------------------------------------------------
TARGET_MAP      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map
LD_FLAGS     = --specs=nano.specs \
              $(ARCH_FLAGS) \
              -lm \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T $(LD_SCRIPT)
              
MCU_COMMON_SRC := \
	drivers/serial_uart_bmf.c \
	drivers/time.c \
	$(SAM_LIB_PATH)/ASF_Support/clock_support.c \
	$(SAM_LIB_PATH)/ASF_Support/spi_support.c \
	$(SAM_LIB_PATH)/ASF_Support/i2c_support.c \
	$(SAM_LIB_PATH)/ASF_Support/tc_support.c \
	$(SAM_LIB_PATH)/ASF_Support/usart_support.c \
	
MCU_COMMON_SRC := 	$(MCU_COMMON_SRC)\
					$(ASF_CSRCS)
					