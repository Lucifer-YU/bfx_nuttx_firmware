
.PHONY: all config archives clean

all:
	make -C ./Firmware CONFIGS=stm32f4discovery-FW_default all

config:
	make -C ./Firmware BOARDS=stm32f4discovery-FW menuconfig

archives:
	make -C ./Firmware BOARDS=stm32f4discovery-FW archives
	
clean: 
	make -C ./Firmware clean

distclean:
	make -C ./Firmware distclean

