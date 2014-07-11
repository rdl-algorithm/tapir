# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------

# Directory for IWYU
IWYUDIR_$(n)  := $(ROOT)/iwyu-out/$(DIR_RELATIVE_$(n))

# File patterns
# Without the extension.
_IWYU2_CPP_$(n)  := $(IWYUDIR_$(n))/%.cpp.iwyu
_IWYU2_C_$(n)    := $(IWYUDIR_$(n))/%.c.iwyu
# With the extension
_IWYU_$(n)       := $(IWYUDIR_$(n))/%.iwyu
_IWYU_FIX_$(n)   := $(IWYUDIR_$(n))/%.iwyu.FIX
_IWYU_FIXED_$(n) := $(IWYUDIR_$(n))/%
# Phony beautification target patterns
_BEAUTIFY_$(n)   := $(SRCDIR_$(n))/%.beautify
_IWYU_FORCE_$(n) := $(IWYUDIR_$(n))/%.iwyu.FORCE
_IWYU_DOFIX_$(n) := $(IWYUDIR_$(n))/%.iwyu.DOFIX

# Include the beautification directories for easy cleanup.
ALL_DIRS_$(n) += $(IWYUDIR_$(n))

# Derived source variables to help identify code cleaning targets.
# 2 => paired header + source
HDRS2_CPP_$(n)   := $(filter $(HDRS_CPP_$(n)),$(patsubst $(_CPP_$(n)),$(_HPP_$(n)),$(SRCS_CPP_$(n))))
SRCS2_CPP_$(n)   := $(patsubst $(_HPP_$(n)),$(_CPP_$(n)),$(HDRS2_CPP_$(n)))

HDRS2_C_$(n)     := $(filter $(HDRS_C_$(n)),$(patsubst $(_C_$(n)),$(_H_$(n)),$(SRCS_C_$(n))))
SRCS2_C_$(n)     := $(patsubst $(_H_$(n)),$(_C_$(n)),$(HDRS2_C_$(n)))

HDRS2_$(n)       := $(HDRS2_CPP_$(n)) $(HDRS2_C_$(n))
SRCS2_$(n)       := $(SRCS2_CPP_$(n)) $(SRCS2_C_$(n))

# 1 => lone header / source
HDRS1_CPP_$(n)   := $(filter-out $(HDRS2_CPP_$(n)),$(HDRS_CPP_$(n)))
SRCS1_CPP_$(n)   := $(filter-out $(SRCS2_CPP_$(n)),$(SRCS_CPP_$(n)))

HDRS1_C_$(n)     := $(filter-out $(HDRS2_C_$(n)),$(HDRS_C_$(n)))
SRCS1_C_$(n)     := $(filter-out $(SRCS2_C_$(n)),$(SRCS_C_$(n)))

HDRS1_$(n)       := $(HDRS1_CPP_$(n)) $(HDRS1_C_$(n))
SRCS1_$(n)       := $(SRCS1_CPP_$(n)) $(SRCS1_C_$(n))

# The actual files.
# Derived source variables for code cleaning patterns.
IWYU2_CPP_$(n)  := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(SRCS2_CPP_$(n)))
IWYU1_CPP_$(n)  := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(HDRS1_CPP_$(n)) $(SRCS1_CPP_$(n)))
IWYU2_C_$(n)    := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(SRCS2_C_$(n)))
IWYU1_C_$(n)    := $(patsubst $(_CODE_$(n)),$(_IWYU_$(n)),$(HDRS1_C_$(n)) $(SRCS1_C_$(n)))

IWYU2_$(n)       := $(IWYU2_CPP_$(n)) $(IWYU2_C_$(n))
IWYU1_$(n)       := $(IWYU1_CPP_$(n)) $(IWYU1_C_$(n))
IWYU_$(n)       := $(IWYU2_$(n)) $(IWYU1_$(n))

IWYU_FIX_$(n)   := $(patsubst $(_IWYU_$(n)),$(_IWYU_FIX_$(n)),$(IWYU_$(n)))
IWYU_FIXED_$(n) := $(patsubst $(_CODE_$(n)),$(_IWYU_FIXED_$(n)),$(CODE_$(n)))

# Phony beautification files!
BCODE_$(n)      := $(filter-out $(BEAUTIFY_EXCLUDES), $(CODE_$(n)))
BEAUTIFY_$(n)   := $(patsubst $(_CODE_$(n)),$(_BEAUTIFY_$(n)),$(BCODE_$(n)))
IWYU_FORCE_$(n) := $(patsubst $(_CODE_$(n)),$(_IWYU_FORCE_$(n)),$(CODE_$(n)))
IWYU_DOFIX_$(n) := $(patsubst $(_CODE_$(n)),$(_IWYU_DOFIX_$(n)),$(CODE_$(n)))

# Directory prerequsites
$(IWYU_$(n)) $(IWYU_FIX_$(n)) $(IWYU_FIXED_$(n)): | $(IWYUDIR_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Add beautification targets as a prerequisite for the "all" targets.
.PHONY: beautify-$(n) iwyu-$(n) iwyu-fix-$(n) iwyu-force-$(n) iwyu-clean-$(n)
beautify-all: beautify-$(n)
iwyu-all: iwyu-$(n)
iwyu-fix-all: iwyu-fix-$(n)
iwyu-force-all: iwyu-force-$(n)
iwyu-dofix-all: iwyu-dofix-$(n)
iwyu-clean-all: iwyu-clean-$(n)

# Clean IWYU when you clean this module.
clean-$(n): iwyu-clean-$(n)
define clean_recipes
iwyu-clean-$(n):
	@rm -f $$(IWYU_$(n)) $$(IWYU_FIX_$(n)) $$(IWYU_FIXED_$(n))
endef
$(eval $(clean_recipes))

# Beautification of this module.
beautify-$(n): $(BEAUTIFY_$(n)) ;
iwyu-$(n): $(IWYU_$(n)) ;
iwyu-fix-$(n): $(IWYU_FIX_$(n)) ;
iwyu-force-$(n): $(IWYU_FORCE_$(n)) ;
iwyu-dofix-$(n): $(IWYU_DOFIX_$(n)) ;

# ----------------------------------------------------------------------
# Beautification recipes
# ----------------------------------------------------------------------
.PHONY: $(BEAUTIFY_$(n))
$(BEAUTIFY_$(n)): $(_BEAUTIFY_$(n)): $(_CODE_$(n))
	$(BEAUTIFY_RECIPE)

$(IWYU2_CPP_$(n)): $(_IWYU2_CPP_$(n)): $(_CPP_$(n)) $(_HPP_$(n))
	$(IWYU_CXX_RECIPE)
$(IWYU1_CPP_$(n)): $(_IWYU_$(n)): $(_CODE_$(n))
	$(IWYU_CXX_RECIPE)

$(IWYU2_C_$(n)): $(_IWYU2_C_$(n)): $(_C_$(n)) $(_H_$(n))
	$(IWYU_CC_RECIPE)
$(IWYU1_C_$(n)): $(_IWYU_$(n)): $(_CODE_$(n))
	$(IWYU_CC_RECIPE)

$(IWYU_FIX_$(n)): $(_IWYU_FIX_$(n)): $(_IWYU_$(n))
	$(IWYU_FIX_RECIPE)

.PHONY: $(IWYU_FORCE_$(n))
$(IWYU_FORCE_$(n)): $(_IWYU_FORCE_$(n)): $(_CODE_$(n))
	$(IWYU_FORCE_RECIPE)

.PHONY: $(IWYU_DOFIX_$(n))
$(IWYU_DOFIX_$(n)): $(_IWYU_DOFIX_$(n)): $(_CODE_$(n))
	$(IWYU_DOFIX_RECIPE)
