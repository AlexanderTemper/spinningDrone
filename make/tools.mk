###############################################################
#
# Installers for tools
#
# NOTE: These are not tied to the default goals
#       and must be invoked manually
#
###############################################################

##############################
#
# Check that environmental variables are sane
#
##############################

# Set up ARM (STM32) SDK
ARM_SDK_DIR ?= $(TOOLS_DIR)/gcc-arm-none-eabi-7-2018-q2-update
# Checked below, Should match the output of $(shell arm-none-eabi-gcc -dumpversion)
GCC_REQUIRED_VERSION ?= 7.3.1

.PHONY: arm_sdk_version

arm_sdk_version:
	$(V1) $(ARM_SDK_PREFIX)gcc --version


# Set up uncrustify tools
UNCRUSTIFY_DIR := $(TOOLS_DIR)/uncrustify-0.61
UNCRUSTIFY_BUILD_DIR := $(DL_DIR)/uncrustify

.PHONY: uncrustify_install
uncrustify_install: | $(DL_DIR) $(TOOLS_DIR)
uncrustify_install: UNCRUSTIFY_URL := http://downloads.sourceforge.net/project/uncrustify/uncrustify/uncrustify-0.61/uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_FILE := uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_OPTIONS := prefix=$(UNCRUSTIFY_DIR)
uncrustify_install: uncrustify_clean
ifneq ($(OSFAMILY), windows)
	@echo " DOWNLOAD     $(UNCRUSTIFY_URL)"
	$(V1) curl -L -k -o "$(DL_DIR)/$(UNCRUSTIFY_FILE)" "$(UNCRUSTIFY_URL)"
endif
        # extract the src
	@echo " EXTRACT      $(UNCRUSTIFY_FILE)"
	$(V1) tar -C $(TOOLS_DIR) -xf "$(DL_DIR)/$(UNCRUSTIFY_FILE)"

	@echo " BUILD        $(UNCRUSTIFY_DIR)"
	$(V1) ( \
	  cd $(UNCRUSTIFY_DIR) ; \
	  ./configure --prefix="$(UNCRUSTIFY_DIR)" ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)
	      # delete the extracted source when we're done
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

.PHONY: uncrustify_clean
uncrustify_clean:
	@echo " CLEAN        $(UNCRUSTIFY_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_DIR)"
	@echo " CLEAN        $(UNCRUSTIFY_BUILD_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"


##############################
#
# Set up paths to tools
#
##############################

ifeq ($(shell [ -d "$(ARM_SDK_DIR)" ] && echo "exists"), exists)
  ARM_SDK_PREFIX := $(ARM_SDK_DIR)/bin/arm-none-eabi-
else ifeq (,$(findstring _install,$(MAKECMDGOALS)))
  GCC_VERSION = $(shell arm-none-eabi-gcc -dumpversion)
  ifeq ($(GCC_VERSION),)
    $(error **ERROR** arm-none-eabi-gcc not in the PATH. Run 'make arm_sdk_install' to install automatically in the tools folder of this repo)
  else ifneq ($(GCC_VERSION), $(GCC_REQUIRED_VERSION))
    $(error **ERROR** your arm-none-eabi-gcc is '$(GCC_VERSION)', but '$(GCC_REQUIRED_VERSION)' is expected. Override with 'GCC_REQUIRED_VERSION' in make/local.mk or run 'make arm_sdk_install' to install the right version automatically in the tools folder of this repo)
  endif

  # ARM tookchain is in the path, and the version is what's required.
  ARM_SDK_PREFIX ?= arm-none-eabi-
endif

ifeq ($(shell [ -d "$(ZIP_DIR)" ] && echo "exists"), exists)
  export ZIPBIN := $(ZIP_DIR)/zip
else
  export ZIPBIN := zip
endif

ifeq ($(shell [ -d "$(OPENOCD_DIR)" ] && echo "exists"), exists)
  OPENOCD := $(OPENOCD_DIR)/bin/openocd
else
  # not installed, hope it's in the path...
  OPENOCD ?= openocd
endif

ifeq ($(shell [ -d "$(UNCRUSTIFY_DIR)" ] && echo "exists"), exists)
  UNCRUSTIFY := $(UNCRUSTIFY_DIR)/bin/uncrustify
else
  # not installed, hope it's in the path...
  UNCRUSTIFY ?= uncrustify
endif
