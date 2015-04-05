
#
# Path settings
#

#
# Some useful paths.
#
#
export NUTTX_SRC := /cygdrive/t/dev/nuttx-code/nuttx

export FW_INCLUDE_DIR	= $(abspath $(FW_BASE)/src/include)
export FW_MODULE_SRC	= $(abspath $(FW_BASE)/src)
export FW_LIB_DIR		= $(abspath $(FW_BASE)/src/lib)
export FW_MK_DIR		= $(abspath $(FW_BASE)/mk)
#export NUTTX_SRC		= $(abspath $(FW_BASE)/NuttX/nuttx)
export MAVLINK_SRC	= $(abspath $(FW_BASE)/mavlink)
export ROMFS_SRC		= $(abspath $(FW_BASE)/ROMFS)
export IMAGE_DIR		= $(abspath $(FW_BASE)/Images)
export BUILD_DIR		= $(abspath $(FW_BASE)/Build)
export ARCHIVE_DIR	= $(abspath $(FW_BASE)/Archives)

#
# Default include paths
#
export INCLUDE_DIRS	:= \
		$(FW_MODULE_SRC) \
		$(FW_MODULE_SRC)/modules/ \
		$(FW_INCLUDE_DIR) \
		$(FW_LIB_DIR)

#
# Tools
#
export COPY			= cp
export COPYDIR		= cp -Rf
export REMOVE		= rm -f
export RMDIR		= rm -rf
export GENROMFS		= genromfs
export TOUCH		= touch
export MKDIR		= mkdir
export FIND			= find
export ECHO			= echo
export UNZIP_CMD		= unzip
export PYTHON		= python
export OPENOCD		= openocd

export GREP = grep

#
# Host-specific paths, hacks and fixups
#
#export SYSTYPE		:= $(shell uname -s)
export SYSTYPE		:= $(shell uname -o)
