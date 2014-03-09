# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------

n := $(MODULE_NAME)

DIR_RELATIVE_$(n) := $(patsubst /%,%,$(subst $(ROOT)/src,,$(d)))

# Source folder
SRCDIR_$(n)   := $(d)

# Source code files (automatic)
HDRS_$(n)     := $(wildcard $(SRCDIR_$(n))/*.hpp)
SRCS_$(n)     := $(wildcard $(SRCDIR_$(n))/*.cpp)
CODE_$(n)     := $(HDRS_$(n)) $(SRCS_$(n))

# File patterns:
# Without cpp/hpp extension
_HPP_$(n)     := $(SRCDIR_$(n))/%.hpp
_CPP_$(n)     := $(SRCDIR_$(n))/%.cpp
# With cpp/hpp extension
_CODE_$(n)    := $(SRCDIR_$(n))/%
# Outputs for easy cleanup.
ALL_DIRS_$(n) :=


# Configuration-specific variables and patterns
BUILDDIR_$(n)    := $(BUILDDIR)/$(DIR_RELATIVE_$(n))
BINDIR_$(n)      := $(BUILDDIR_$(n))
OBJDIR_$(n)      := $(BUILDDIR_$(n))
ALL_DIRS_$(n)    += $(BUILDDIR_$(n))

_BIN_$(n)        := $(BINDIR_$(n))/%
_O_$(n)          := $(OBJDIR_$(n))/%.o
_D_$(n)          := $(OBJDIR_$(n))/%.d

OBJS_$(n)        := $(patsubst $(_CPP_$(n)),$(_O_$(n)),$(SRCS_$(n)))
DEPS_$(n)        := $(patsubst $(_O_$(n)),$(_D_$(n)),$(OBJS_$(n)))
OUTPUTS_TO_CLEAN_$(n)  = $(OBJS_$(n)) $(DEPS_$(n))

# Directory prerequsites
$(OBJS_$(n)):| $(OBJDIR_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

.PHONY: $(n) build-$(n) clean-$(n)
# Use "build" as the default command
$(n): build-$(n)
# Add local targets as prerequisites for the grouped targets
clean-all: clean-$(n)
build-all: build-$(n)


# Directory recipe
$(ALL_DIRS_$(n)):
	$(MKDIR_RECIPE)

-include $(DEPS_$(n))
# Compilation recipe
$(OBJS_$(n)): $(_O_$(n)): $(_CPP_$(n))
	$(COMPILE_CMD) $<

# Rebuild if this file changes, or the root makefile does.
$(OBJS_$(n)): Makefile .make/template.mk

# The "clean" command cleans all configurations
define clean_recipe
clean-$(n):
	@rm -f $(OUTPUTS_TO_CLEAN_$(n))
	@echo Cleaned $(n)
endef
$(eval $(clean_recipe))

# Extra (but non-essential) rules for beautifying the code!
include .make/beautify.mk
