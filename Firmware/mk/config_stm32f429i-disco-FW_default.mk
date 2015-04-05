

#
# Board support modules
#
MODULES		+= modules/led_marquee
MODULES		+= drivers/device
MODULES		+= drivers/hmc5883l

#
# System commands
#


#
# General system control
#

#
# Library modules
#
MODULES		+= lib/system

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define DEF_CMD_ENTRY
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

# command, priority, stack, entrypoint
BUILTIN_COMMANDS := \
#	$(call DEF_CMD_ENTRY, sercon, , 2048, sercon_main) \
#	$(call DEF_CMD_ENTRY, serdis, , 2048, serdis_main)
