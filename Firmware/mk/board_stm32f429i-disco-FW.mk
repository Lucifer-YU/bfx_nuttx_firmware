#
# Board-specific definitions
#

#
# Configure the toolchain
#
CONFIG_ARCH	= CORTEXM4F
CONFIG_BOARD	= STM32F429I_DISCO

LIBS += $(WORK_DIR)/nuttx-export/startup/stm32_vectors.o

include $(FW_MK_DIR)/toolchain_gnu-arm-eabi.mk
