
################################################################################
# No user-serviceable parts below.
################################################################################

ifeq ($(MODULE_MK),)
$(error No module makefile specified)
endif
$(info %% MODULE_MK = $(MODULE_MK))

#
# Get the board/toolchain config
#
include $(BOARD_FILE)

#
# Get the module's config
#
include $(MODULE_MK)
MODULE_SRC := $(dir $(MODULE_MK))
$(info %  MODULE_NAME		= $(MODULE_NAME))
$(info %  MODULE_SRC		= $(MODULE_SRC))
$(info %  MODULE_OBJ		= $(MODULE_OBJ))
$(info %  MODULE_WORK_DIR	= $(MODULE_WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS += $(MAKEFILE_LIST)

################################################################################
# Builtin command definitions
################################################################################

ifneq ($(MODULE_COMMAND),)
MODULE_ENTRYPOINT	?= $(MODULE_COMMAND)_main
MODULE_STACKSIZE	?= CONFIG_PTHREAD_STACK_DEFAULT
MODULE_PRIORITY	?= SCHED_PRIORITY_DEFAULT
MODULE_COMMANDS	+= $(MODULE_COMMAND).$(MODULE_PRIORITY).$(MODULE_STACKSIZE).$(MODULE_ENTRYPOINT)
endif

ifneq ($(MODULE_COMMANDS),)
MODULE_COMMAND_FILES	:= $(addprefix $(WORK_DIR)/builtin_commands/COMMAND.,$(MODULE_COMMANDS))

# Create the command files
# Ensure that there is only one entry for each command
#
.PHONY: $(MODULE_COMMAND_FILES)
$(MODULE_COMMAND_FILES): command = $(word 2,$(subst ., ,$(notdir $(@))))
$(MODULE_COMMAND_FILES): exclude = $(dir $@)COMMAND.$(command).*
$(MODULE_COMMAND_FILES): $(GLOBAL_DEPS)
	@$(ECHO) ========== Creating command file: $@
	@$(REMOVE) -f $(exclude)
	@$(MKDIR) -p $(dir $@)
	@$(ECHO) "CMD:     $(command)"
	$(TOUCH) $@
endif

################################################################################
# Adjust compilation flags to implement EXPORT
################################################################################

ifeq ($(DEFAULT_VISIBILITY),)
DEFAULT_VISIBILITY = hidden
else
DEFAULT_VISIBILITY = default
endif

CFLAGS		+= -fvisibility=$(DEFAULT_VISIBILITY) -include $(FW_INCLUDE_DIR)/visibility.h
CXXFLAGS	+= -fvisibility=$(DEFAULT_VISIBILITY) -include $(FW_INCLUDE_DIR)/visibility.h

################################################################################
# Build rules
################################################################################

#
# What we're going to build
#
module:			$(MODULE_OBJ) $(MODULE_COMMAND_FILES)

#
# Object files we will generate from sources
#
OBJS			 = $(addsuffix .o,$(SRCS))

#
# Dependency files that will be auto-generated
#
DEPS			 = $(addsuffix .d,$(SRCS))

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

vpath %.c $(MODULE_SRC)
$(filter %.c.o,$(OBJS)): %.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

vpath %.cpp $(MODULE_SRC)
$(filter %.cpp.o,$(OBJS)): %.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

vpath %.S $(MODULE_SRC)
$(filter %.S.o,$(OBJS)): %.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#

$(MODULE_OBJ): $(OBJS) $(GLOBAL_DEPS)
	@$(ECHO) ========== Generating module obj file: $@
	$(call PRELINK,$@,$(OBJS))

#
# Utility rules
#

clean:
	$(REMOVE) $(MODULE_PRELINK) $(OBJS)

-include $(DEPS)
