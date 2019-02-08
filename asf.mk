# Path to top level ASF directory relative to this project directory.
ASF_PATH = src/ASF

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

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = $(ASF_PATH)/sam0/utils/linker_scripts/samd20/gcc/samd20j18_flash.ld




