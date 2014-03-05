# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------

n := $(MODULE_NAME)

DIR_RELATIVE_$(n) := $(subst $(ROOT)/src,,$(d))

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
_CODE_$(n)       := $(SRCDIR_$(n))/%
# Outputs for easy cleanup.
ALL_DIRS_$(n) :=


# Configuration-specific variables and patterns
define build_defaults
BUILDDIR_$(n)_$(1)    := $(ROOT)/builds/$(1)/$(DIR_RELATIVE_$(n))
BINDIR_$(n)_$(1)      := $$(BUILDDIR_$(n)_$(1))
OBJDIR_$(n)_$(1)      := $$(BUILDDIR_$(n)_$(1))
ALL_DIRS_$(n)         += $$(BUILDDIR_$(n)_$(1))

_BIN_$(n)_$(1)        := $$(BINDIR_$(n)_$(1))/%
_O_$(n)_$(1)          := $$(OBJDIR_$(n)_$(1))/%.o
_D_$(n)_$(1)          := $$(OBJDIR_$(n)_$(1))/%.d

OBJS_$(n)_$(1)        := $$(patsubst $(_CPP_$(n)),$$(_O_$(n)_$(1)),$(SRCS_$(n)))
DEPS_$(n)_$(1)        := $$(patsubst $$(_O_$(n)_$(1)),$$(_D_$(n)_$(1)),$$(OBJS_$(n)_$(1)))
OUTPUTS_TO_CLEAN_$(n)_$(1)  = $$(OBJS_$(n)_$(1)) $$(DEPS_$(n)_$(1))

# Directory prerequsites
$$(OBJS_$(n)_$(1)):       | $$(OBJDIR_$(n)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call build_defaults,$(cfg))))

ALL_DIRS_$(n) += $(BUILDDIR_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

.PHONY: $(n) build-$(n) clean-$(n)
# Default builds
$(n) build-$(n): build-$(DEFAULT_CFG)-$(n) ;
# Add local targets as prerequisites for the grouped targets
clean-all: clean-$(n)

# Directory recipe
$(ALL_DIRS_$(n)):
	$(MKDIR_RECIPE)

# Configuration-specific-rules
define conf_rules
-include $(DEPS_$(n)_$(1))
$(OBJS_$(n)_$(1)): $(_O_$(n)_$(1)): $(_CPP_$(n))
	$$(COMPILE_CMD_$(1)) $$<

# Rebuild if this file changes, or the root makefile does.
$(OBJS_$(n)_$(1)): Makefile .make/template.mk

.PHONY: $(1)-$(n) build-$(1)-$(n) clean-$(1)-$(n)
# Add local targets as prerequisites for the grouped targets
clean-$(1)-all: clean-$(1)-$(n)
build-$(1)-all: build-$(1)-$(n)

# Use "build" as the default command
$(1)-$(n): build-$(1)-$(n)

# The "clean" command cleans all configurations
clean-$(n): clean-$(1)-$(n)
clean-$(1)-$(n):
	@rm -f $$(OUTPUTS_TO_CLEAN_$(n)_$(1))
	@echo Cleaned $(1)-$(n)
endef
$(foreach cfg,$(CFGS),$(eval $(call conf_rules,$(cfg))))

# Extra (but non-essential) rules for beautifying the code!
include .make/beautify.mk
