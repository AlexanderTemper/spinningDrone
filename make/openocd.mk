OPENOCD ?= openocd
OPENOCD_IF ?= interface/stlink-v2.cfg

ifeq ($(TARGET_MCU),STM32F3)
OPENOCD_CFG := target/stm32f3x.cfg

else ifeq ($(TARGET_MCU),STM32F4)
OPENOCD_CFG := target/stm32f4x.cfg

else ifeq ($(TARGET_MCU),STM32F7)
OPENOCD_CFG := target/stm32f7x.cfg

else ifeq ($(TARGET_MCU),SAMD20J18)
OPENOCD_IF = interface/cmsis-dap.cfg
OPENOCD_CFG := target/at91samdXX.cfg
else
endif

ifneq ($(OPENOCD_CFG),)
OPENOCD_COMMAND = $(OPENOCD) -f $(OPENOCD_IF) -f $(OPENOCD_CFG)
endif
