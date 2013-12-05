# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------

# Source and IWYU folders
SRCDIR_$(n)   := $(d)/src
BUILDDIR_$(n) := $(d)/builds
IWYUDIR_$(n)  := $(d)/iwyu-out

# Source code files (automatic)
HDRS_$(n)     := $(wildcard $(SRCDIR_$(n))/*.hpp)
SRCS_$(n)     := $(wildcard $(SRCDIR_$(n))/*.cpp)

# File patterns
_HPP_$(n)     := $(SRCDIR_$(n))/%.hpp
_CPP_$(n)     := $(SRCDIR_$(n))/%.cpp
_CODE_$(n)    := $(SRCDIR_$(n))/%
_IWYU2_$(n)   := $(IWYUDIR_$(n))/%.cpp.iwyu
_IWYU1_$(n)   := $(IWYUDIR_$(n))/%.iwyu
_IWYU_$(n)    := $(IWYUDIR_$(n))/%.iwyu
_IWYU_FIX_$(n) := $(IWYUDIR_$(n))/%.iwyu.FIX

# Outputs for easy cleanup.
ALL_DIRS_$(n) := $(IWYUDIR_$(n))

# Configuration-specific variables and patterns
define build_defaults
BUILDDIR_$(n)_$(1)    := $(BUILDDIR_$(n))/$(1)
BINDIR_$(n)_$(1)      := $$(BUILDDIR_$(n)_$(1))
OBJDIR_$(n)_$(1)      := $$(BUILDDIR_$(n)_$(1))
ALL_DIRS_$(n)         += $$(BUILDDIR_$(n)_$(1))

_BIN_$(n)_$(1)        := $$(BINDIR_$(n)_$(1))/%
_O_$(n)_$(1)          := $$(OBJDIR_$(n)_$(1))/%.o
_D_$(n)_$(1)          := $$(OBJDIR_$(n)_$(1))/%.d

OBJS_$(n)_$(1)        := $$(patsubst $(_CPP_$(n)),$$(_O_$(n)_$(1)),$(SRCS_$(n)))
DEPS_$(n)_$(1)        := $$(patsubst $$(_O_$(n)_$(1)),$$(_D_$(n)_$(1)),$$(OBJS_$(n)_$(1)))
ALL_OUTPUTS_$(n)_$(1)  = $$(OBJS_$(n)_$(1)) $$(DEPS_$(n)_$(1))

# Directory prerequsites
$$(BUILDDIR_$(n)_$(1)):   | $(BUILDDIR_$(n))
$$(OBJS_$(n)_$(1)):       | $$(OBJDIR_$(n)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call build_defaults,$(cfg))))

ALL_DIRS_$(n) += $(BUILDDIR_$(n))

#Derived source variables for IWYU patterns.
# 2 => paired hpp/cpp
# 1 => lone hpp or cpp
HDRS2_$(n)   := $(filter $(HDRS_$(n)),$(patsubst $(_CPP_$(n)),$(_HPP_$(n)),$(SRCS_$(n))))
SRCS2_$(n)   := $(patsubst $(_HPP_$(n)),$(_CPP_$(n)),$(HDRS2_$(n)))
IWYU2_$(n)   := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(SRCS2_$(n)))

HDRS1_$(n)   := $(filter-out $(HDRS2_$(n)),$(HDRS_$(n)))
SRCS1_$(n)   := $(filter-out $(SRCS2_$(n)),$(SRCS_$(n)))
IWYU1_$(n)   := $(patsubst $(_CODE_$(n)),$(_IWYU1_$(n)),$(HDRS1_$(n)) $(SRCS1_$(n)))

IWYU_$(n)    := $(IWYU2_$(n)) $(IWYU1_$(n))
IWYU_FIX_$(n) := $(patsubst $(_IWYU_$(n)),$(_IWYU_FIX_$(n)),$(IWYU_$(n)))

# Directory prerequsites
$(IWYU_$(n)) $(IWYU_FIX_$(n)): | $(IWYUDIR_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# General-purpose phony targets
.PHONY: $(n) build-$(n) clean-$(n) rmdirs-$(n)
.PHONY: iwyu-$(n) iwyu-fix-$(n) iwyu-clean-$(n)

# Default builds
$(n) build-$(n): build-$(DEFAULT_CFG)-$(n) ;

# Add local targets as prerequisites for the grouped targets
clean-all: clean-$(n)
rmdirs-all: rmdirs-$(n)
iwyu-all: iwyu-$(n)
iwyu-fix-all: iwyu-fix-$(n)
iwyu-clean-all: iwyu-clean-$(n)

# Directory recipe
$(ALL_DIRS_$(n)):
	$(MKDIR_RECIPE)

clean-$(n): iwyu-clean-$(n)
define clean_recipes
iwyu-clean-$(n):
	@rm -f $$(IWYU_$(n)) $$(IWYU_FIX_$(n))
rmdirs-$(n): clean-$(n)
	@rm -fd $$(ALL_DIRS_$(n))
endef
$(eval $(clean_recipes))

# Configuration-specific-rules
define conf_rules
-include $(DEPS_$(n)_$(1))
$(OBJS_$(n)_$(1)): $(_O_$(n)_$(1)): $(_CPP_$(n))
	$$(COMPILE_RECIPE_$(1))

.PHONY: $(1)-$(n) build-$(1)-$(n) clean-$(1)-$(n)
# Add local targets as prerequisites for the grouped targets
clean-$(1)-all: clean-$(1)-$(n)
build-$(1)-all: build-$(1)-$(n)

# Use "build" as the default command
$(1)-$(n): build-$(1)-$(n)

# The "clean" command cleans all configurations
clean-$(n): clean-$(1)-$(n)
clean-$(1)-$(n):
	@rm -f $$(ALL_OUTPUTS_$(n)_$(1))
	@echo Cleaned $(1)-$(n)
endef
$(foreach cfg,$(CFGS),$(eval $(call conf_rules,$(cfg))))

# IWYU rules

# IWYU and IWYU-FIX require all the files to be checked
iwyu-$(n): $(IWYU_$(n)) ;
iwyu-fix-$(n): $(IWYU_FIX_$(n)) ;

# IWYU recipes
$(IWYU2_$(n)): $(_IWYU2_$(n)): $(_CPP_$(n)) $(_HPP_$(n))
	$(IWYU_RECIPE)
$(IWYU1_$(n)): $(_IWYU1_$(n)): $(_CODE_$(n))
	$(IWYU_RECIPE)
$(IWYU_FIX_$(n)): $(_IWYU_FIX_$(n)): $(_IWYU_$(n))
	$(IWYU_FIX_RECIPE)
