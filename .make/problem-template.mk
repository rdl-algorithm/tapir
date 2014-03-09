include .make/template.mk

TARGET_NAMES_$(n) := $(TARGET_NAMES)

# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------
TGTS_$(n)             := $(patsubst %,$(_BIN_$(n)),$(TARGET_NAMES_$(n)))
OUTPUTS_TO_CLEAN_$(n) += $(TGTS_$(n))
OBJS_TGT_$(n)         := $(patsubst $(_BIN_$(n)),$(_O_$(n)),$(TGTS_$(n)))
OBJS_NTGT_$(n)        := $(filter-out $(OBJS_TGT_$(n)),$(OBJS_$(n)))

# Directory prerequsites
$(TGTS_$(n)): | $(BINDIR_$(n))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Dependencies for the linker
LINKER_DEPS_$(n) := $(EXTRA_LINKER_DEPS_$(n)) $(OBJS_NTGT_$(n))
LINKER_DEPS_$(n) += $$(LIB_solver)
LINKER_DEPS_$(n) += -lboost_program_options -lspatialindex

# Linking rule for the executable targets.
define problem_build_template
$(TGTS_$(n)): $(_BIN_$(n)): $(_O_$(n)) $$(LINKER_DEPS_$(n))
	$$(LINK_CMD) $$< $(LINKER_DEPS_$(n))

# Executables will be copied to the main folder for convenience
build-$(n): $$(TGTS_$(n))
	cp $$^ $(d)

# Rebuild if this makefile changes.
$(TGTS_$(n)): .make/problem-template.mk
endef
$(eval $(problem_build_template))

# To clean we must also clean the copied executables.
.PHONY: clean-mains-$(n)
clean-$(n): clean-mains-$(n)
MAINS_$(n) := $(addprefix $(d)/,$(TARGET_NAMES_$(n)))
define clean_mains_recipe
clean-mains-$(n):
	@rm -f $(MAINS_$(n))
endef
$(eval $(clean_mains_recipe))
