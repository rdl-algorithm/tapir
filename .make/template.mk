# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------

n := $(MODULE_NAME)
DIR_RELATIVE_$(n) := $(patsubst /%,%,$(subst $(ROOT)/src,,$(d)))

# Source folder
SRCDIR_$(n)   := $(d)

# Source code files (automatic)
HDRS_CPP_$(n)  := $(wildcard $(SRCDIR_$(n))/*.hpp)
SRCS_CPP_$(n)  := $(wildcard $(SRCDIR_$(n))/*.cpp)
HDRS_C_$(n)    := $(wildcard $(SRCDIR_$(n))/*.h)
SRCS_C_$(n)    := $(wildcard $(SRCDIR_$(n))/*.c)
CODE_$(n)      := $(HDRS_CPP_$(n)) $(HDRS_C_$(n)) $(SRCS_CPP_$(n)) $(SRCS_C_$(n))

# Source file patterns:
# Without extension
_HPP_$(n)     := $(SRCDIR_$(n))/%.hpp
_CPP_$(n)     := $(SRCDIR_$(n))/%.cpp
_H_$(n)       := $(SRCDIR_$(n))/%.h
_C_$(n)       := $(SRCDIR_$(n))/%.c
# With extension
_CODE_$(n)    := $(SRCDIR_$(n))/%


# Configuration-specific variables and patterns
BUILDDIR_$(n)    := $(BUILDDIR)/$(DIR_RELATIVE_$(n))
OBJDIR_$(n)      := $(BUILDDIR_$(n))
ALL_DIRS_$(n)    := $(BUILDDIR_$(n))

_O_$(n)          := $(OBJDIR_$(n))/%.o
_D_$(n)          := $(OBJDIR_$(n))/%.d

OBJS_CPP_$(n)    := $(patsubst $(_CPP_$(n)),$(_O_$(n)),$(SRCS_CPP_$(n)))
OBJS_C_$(n)      := $(patsubst $(_C_$(n)),$(_O_$(n)),$(SRCS_C_$(n)))
OBJS_$(n)        := $(OBJS_CPP_$(n)) $(OBJS_C_$(n))

DEPS_$(n)        := $(patsubst $(_O_$(n)),$(_D_$(n)),$(OBJS_$(n)))
OUTPUTS_TO_CLEAN_$(n)  := $(OBJS_$(n)) $(DEPS_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Automatically include all the dependencies!
-include $(DEPS_$(n))
# Rebuild if this file changes, or the root makefile does.
$(OBJS_$(n)): Makefile .make/template.mk

# Directory prerequsites
$(OBJS_$(n)):| $(OBJDIR_$(n))
# Directory recipe
$(ALL_DIRS_$(n)):
	$(MKDIR_RECIPE)

# Phony targets for this configuration.
.PHONY: $(n) build-$(n) clean-$(n)
# Use "build" as the default command
$(n): build-$(n)
# Add local targets as prerequisites for the grouped targets
clean: clean-$(n)
build-all: build-$(n)

# Compilation recipes, C and C++
$(OBJS_CPP_$(n)): $(_O_$(n)): $(_CPP_$(n))
	$(call CXX_RECIPE,$<)
$(OBJS_C_$(n)): $(_O_$(n)): $(_C_$(n))
	$(call CC_RECIPE,$<)

# The "clean" command cleans all configurations; we use this
# define-and-eval pattern so that that $(n) will be expanded NOW,
# but $$(OUTPUTS_TO_CLEAN_$(n)) will be expanded LATER.
# That way, OUTPUTS_TO_CLEAN can be updated.
define clean_recipe
clean-$(n):
	@rm -f $$(OUTPUTS_TO_CLEAN_$(n))
	@echo Cleaned $(n)
endef
$(eval $(clean_recipe))

# Extra (but non-essential) rules for beautifying the code!
include .make/beautify.mk
