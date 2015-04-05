
#
# Top-level Makefile for building PX4 firmware images.
#

#
# Get path and tool configuration
#
export FW_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

include $(FW_BASE)/mk/setup.mk

#
# Canned firmware configurations that we (know how to) build.
#
KNOWN_CONFIGS := $(subst config_,,$(basename $(notdir $(wildcard $(FW_MK_DIR)/config_*.mk))))
CONFIGS ?= $(KNOWN_CONFIGS)

#
# Boards that we (know how to) build NuttX export kits for.
#
KNOWN_BOARDS := $(subst board_,,$(basename $(notdir $(wildcard $(FW_MK_DIR)/board_*.mk))))
BOARDS ?= $(KNOWN_BOARDS)


#
# If the user has listed a config as a target, strip it out and override CONFIGS.
#
EXPLICIT_CONFIGS := $(filter $(CONFIGS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_CONFIGS),)
CONFIGS := $(EXPLICIT_CONFIGS)
.PHONY: $(EXPLICIT_CONFIGS)
$(EXPLICIT_CONFIGS): all
endif

#
# Built products
#
DESIRED_FIRMWARES = $(foreach config,$(CONFIGS),$(IMAGE_DIR)/$(config).hex)
FIRMWARES = $(foreach config,$(KNOWN_CONFIGS),$(BUILD_DIR)/$(config).build/firmware.hex)

all: $(DESIRED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
$(DESIRED_FIRMWARES): $(IMAGE_DIR)/%.hex: $(BUILD_DIR)/%.build/firmware.hex
	@$(ECHO) %% Copying $@
	$(MKDIR) -p $(IMAGE_DIR)
	$(COPY) $< $@
	$(COPY) $(patsubst %.hex,%.bin,$<) $(patsubst %.hex,%.bin,$@)
	$(COPY) $(patsubst %.hex,%.elf,$<) $(patsubst %.hex,%.elf,$@)

#
# Generate FIRMWARES.
#
.PHONY: $(FIRMWARES)
$(BUILD_DIR)/%.build/firmware.hex: config   = $(patsubst $(BUILD_DIR)/%.build/firmware.hex,%,$@)
$(BUILD_DIR)/%.build/firmware.hex: work_dir = $(BUILD_DIR)/$(config).build
$(FIRMWARES): $(BUILD_DIR)/%.build/firmware.hex:
	@$(ECHO) %%%%
	@$(ECHO) %%%% Building $(config) in $(work_dir)
	@$(ECHO) %%%%
	$(MKDIR) -p $(work_dir)
	$(MAKE) -r -C $(work_dir) \
		-f $(FW_MK_DIR)/firmware.mk \
		CONFIG=$(config) \
		WORK_DIR=$(work_dir)

#
# Build the NuttX export archives.
#
NUTTX_ARCHIVES = $(foreach board,$(BOARDS),$(ARCHIVE_DIR)/$(board).export)
.PHONY: archives
archives: $(NUTTX_ARCHIVES)

$(ARCHIVE_DIR)/%.export: board = $(notdir $(basename $@))
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)/%.export: $(NUTTX_SRC)
	@$(ECHO) %% Configuring NuttX for $(board)
	(cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(MAKE) -r -j1 -C $(NUTTX_SRC) -r distclean
	(cd $(NUTTX_SRC)/configs && $(COPYDIR) $(FW_BASE)/nuttx-configs/$(board) .)
	(cd $(NUTTX_SRC)/tools && ./configure.sh $(board)/nsh)
	@$(ECHO) %% Exporting NuttX for $(board)
	$(MAKE) -r -j1 -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(board) export
	$(MKDIR) -p $(dir $@)
	$(COPY) $(NUTTX_SRC)/nuttx-export.zip $@
	(cd $(NUTTX_SRC)/configs && $(RMDIR) $(board))

#
# The user can run the nuttx 'menuconfig' tool for a single board configuration with
# make BOARDS=<boardname> menuconfig
#
ifeq ($(MAKECMDGOALS),menuconfig)
ifneq ($(words $(BOARDS)),1)
$(error BOARDS must specify exactly one board for the menuconfig goal)
endif
BOARD			 = $(BOARDS)
menuconfig: $(NUTTX_SRC)
	@$(ECHO) %% Configuring NuttX for $(BOARD)
	(cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(MAKE) -r -j1 -C $(NUTTX_SRC) -r $(MQUIET) distclean
	(cd $(NUTTX_SRC)/configs && $(COPYDIR) $(FW_BASE)/nuttx-configs/$(BOARD) .)
	(cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/nsh)
	@$(ECHO) %% Running menuconfig for $(BOARD)
	$(MAKE) -r -j1 -C $(NUTTX_SRC) -r $(MQUIET) menuconfig
	@$(ECHO) %% Saving configuration file
	$(COPY) $(NUTTX_SRC)/.config $(FW_BASE)/nuttx-configs/$(BOARD)/nsh/defconfig
else
menuconfig:
	@$(ECHO) ""
	@$(ECHO) "The menuconfig goal must be invoked without any other goal being specified"
	@$(ECHO) ""
endif

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything 
# that's generated leaving only files that are in source control.
#

.PHONY:	clean
clean:
	$(RMDIR) $(BUILD_DIR)/*.build
	$(REMOVE) $(IMAGE_DIR)/*.hex

.PHONY:	distclean
distclean: clean
	$(REMOVE) $(ARCHIVE_DIR)/*.export
	$(MAKE) -C $(NUTTX_SRC) -r distclean
	(cd $(NUTTX_SRC)/configs && $(FIND) . -maxdepth 1 -type l -delete)
