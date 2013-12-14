# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------

DIR_RELATIVE_$(n) := $(subst $(ROOT)/src,,$(d))

# Source and IWYU folders
SRCDIR_$(n)   := $(d)
IWYUDIR_$(n)  := $(ROOT)/iwyu-out/$(DIR_RELATIVE_$(n))

# Source code files (automatic)
HDRS_$(n)     := $(wildcard $(SRCDIR_$(n))/*.hpp)
SRCS_$(n)     := $(wildcard $(SRCDIR_$(n))/*.cpp)
CODE_$(n)     := $(HDRS_$(n)) $(SRCS_$(n))

# File patterns
# Without cpp/hpp extension
_HPP_$(n)     := $(SRCDIR_$(n))/%.hpp
_CPP_$(n)     := $(SRCDIR_$(n))/%.cpp
_IWYU2_$(n)   := $(IWYUDIR_$(n))/%.cpp.iwyu

# With cpp/hpp extension
_CODE_$(n)       := $(SRCDIR_$(n))/%
_IWYU_$(n)       := $(IWYUDIR_$(n))/%.iwyu
_IWYU_FIX_$(n)   := $(IWYUDIR_$(n))/%.iwyu.FIX
_IWYU_FIXED_$(n) := $(IWYUDIR_$(n))/%

# Phony.
_BEAUTIFY_$(n)   := $(SRCDIR_$(n))/%.beautify
_IWYU_FORCE_$(n) := $(IWYUDIR_$(n))/%.iwyu.FORCE
_IWYU_DOFIX_$(n) := $(IWYUDIR_$(n))/%.iwyu.DOFIX

# Outputs for easy cleanup.
ALL_DIRS_$(n) := $(IWYUDIR_$(n))

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
ALL_OUTPUTS_$(n)_$(1)  = $$(OBJS_$(n)_$(1)) $$(DEPS_$(n)_$(1))

# Directory prerequsites
$$(OBJS_$(n)_$(1)):       | $$(OBJDIR_$(n)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call build_defaults,$(cfg))))

ALL_DIRS_$(n) += $(BUILDDIR_$(n))

# Derived source variables for code cleaning patterns.
# 2 => paired hpp/cpp
# 1 => lone hpp or cpp
HDRS2_$(n)     := $(filter $(HDRS_$(n)),$(patsubst $(_CPP_$(n)),$(_HPP_$(n)),$(SRCS_$(n))))
SRCS2_$(n)     := $(patsubst $(_HPP_$(n)),$(_CPP_$(n)),$(HDRS2_$(n)))
IWYU2_$(n)     := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(SRCS2_$(n)))

HDRS1_$(n)   := $(filter-out $(HDRS2_$(n)),$(HDRS_$(n)))
SRCS1_$(n)   := $(filter-out $(SRCS2_$(n)),$(SRCS_$(n)))
IWYU1_$(n)   := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(HDRS1_$(n)) $(SRCS1_$(n)))

IWYU_$(n)       := $(IWYU2_$(n)) $(IWYU1_$(n))
IWYU_FIX_$(n)   := $(patsubst $(_IWYU_$(n)),$(_IWYU_FIX_$(n)),$(IWYU_$(n)))
IWYU_FIXED_$(n) := $(patsubst $(_CODE_$(n)),$(_IWYU_FIXED_$(n)),$(CODE_$(n)))

# Phony!
BCODE_$(n)      := $(filter-out $(BEAUTIFY_EXCLUDES), $(CODE_$(n)))
BEAUTIFY_$(n)   := $(patsubst $(_CODE_$(n)),$(_BEAUTIFY_$(n)),$(BCODE_$(n)))
IWYU_FORCE_$(n) := $(patsubst $(_CODE_$(n)),$(_IWYU_FORCE_$(n)),$(CODE_$(n)))
IWYU_DOFIX_$(n) := $(patsubst $(_CODE_$(n)),$(_IWYU_DOFIX_$(n)),$(CODE_$(n)))

# Directory prerequsites
$(IWYU_$(n)) $(IWYU_FIX_$(n)) $(IWYU_FIXED_$(n)): | $(IWYUDIR_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# General-purpose phony targets
.PHONY: $(n) build-$(n) clean-$(n)
# Default builds
$(n) build-$(n): build-$(DEFAULT_CFG)-$(n) ;
# Add local targets as prerequisites for the grouped targets
clean-all: clean-$(n)

# Code-cleaning targets.
.PHONY: beautify-$(n) iwyu-$(n) iwyu-fix-$(n) iwyu-force-$(n) iwyu-clean-$(n)
beautify-all: beautify-$(n)
iwyu-all: iwyu-$(n)
iwyu-fix-all: iwyu-fix-$(n)
iwyu-force-all: iwyu-force-$(n)
iwyu-dofix-all: iwyu-dofix-$(n)
iwyu-clean-all: iwyu-clean-$(n)

# Directory recipe
$(ALL_DIRS_$(n)):
	$(MKDIR_RECIPE)

clean-$(n): iwyu-clean-$(n)
define clean_recipes
iwyu-clean-$(n):
	@rm -f $$(IWYU_$(n)) $$(IWYU_FIX_$(n)) $$(IWYU_FIXED_$(n))
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


# Code cleaning rules
beautify-$(n): $(BEAUTIFY_$(n)) ;
iwyu-$(n): $(IWYU_$(n)) ;
iwyu-fix-$(n): $(IWYU_FIX_$(n)) ;
iwyu-force-$(n): $(IWYU_FORCE_$(n)) ;
iwyu-dofix-$(n): $(IWYU_DOFIX_$(n)) ;

.PHONY: $(BEAUTIFY_$(n))
$(BEAUTIFY_$(n)): $(_BEAUTIFY_$(n)): $(_CODE_$(n))
	$(BEAUTIFY_RECIPE)

$(IWYU2_$(n)): $(_IWYU2_$(n)): $(_CPP_$(n)) $(_HPP_$(n))
	$(IWYU_RECIPE)
$(IWYU1_$(n)): $(_IWYU_$(n)): $(_CODE_$(n))
	$(IWYU_RECIPE)

$(IWYU_FIX_$(n)): $(_IWYU_FIX_$(n)): $(_IWYU_$(n))
	$(IWYU_FIX_RECIPE)

.PHONY: $(IWYU_FORCE_$(n))
$(IWYU_FORCE_$(n)): $(_IWYU_FORCE_$(n)): $(_CODE_$(n))
	$(IWYU_FORCE_RECIPE)

.PHONY: $(IWYU_DOFIX_$(n))
$(IWYU_DOFIX_$(n)): $(_IWYU_DOFIX_$(n)): $(_CODE_$(n))
	$(IWYU_DOFIX_RECIPE)
