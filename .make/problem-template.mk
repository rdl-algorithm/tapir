include .make/template.mk

TARGET_NAMES_$(n) := $(TARGET_NAMES)

# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------
define problem_defaults
TGTS_$(n)_$(1)        := $(patsubst %,$(_BIN_$(n)_$(1)),$(TARGET_NAMES_$(n)))
OUTPUTS_TO_CLEAN_$(n)_$(1) += $$(TGTS_$(n)_$(1))
OBJS_TGT_$(n)_$(1)    := $$(patsubst $(_BIN_$(n)_$(1)),$(_O_$(n)_$(1)),$$(TGTS_$(n)_$(1)))
OBJS_NTGT_$(n)_$(1)   := $$(filter-out $$(OBJS_TGT_$(n)_$(1)),$(OBJS_$(n)_$(1)))

# Directory prerequsites
$$(TGTS_$(n)_$(1)): | $$(BINDIR_$(n)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call problem_defaults,$(cfg))))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Dependencies for the linker
define dependencies_template
LINKER_DEPS_$(n)_$(1) := $(EXTRA_LINKER_DEPS_$(n)_$(1)) $$$$(OBJS_NTGT_$(n)_$(1))
LINKER_DEPS_$(n)_$(1) += $$$$(LIB_solver_$(1))
LINKER_DEPS_$(n)_$(1) += -lboost_program_options -lspatialindex
endef 
$(foreach cfg,$(CFGS),$(eval $(call dependencies_template,$(cfg))))

# Linking rule for the executable targets.
define problem_build_template
$(TGTS_$(n)_$(1)): $(_BIN_$(n)_$(1)): $(_O_$(n)_$(1)) $$(LINKER_DEPS_$(n)_$(1))
	$$(LINK_CMD_$(1)) $$< $(LINKER_DEPS_$(n)_$(1))

# Executables will be copied to the main folder for convenience
build-$(1)-$(n): $(TGTS_$(n)_$(1))
	cp $$^ $(d)

# Rebuild if this makefile changes.
$(TGTS_$(n)_$(1)): .make/problem-template.mk
endef
$(foreach cfg,$(CFGS),$(eval $(call problem_build_template,$(cfg))))

# To clean we must also clean the copied executables.
.PHONY: clean-mains-$(n)
clean-$(n): clean-mains-$(n)
MAINS_$(n) := $(addprefix $(d)/,$(TARGET_NAMES_$(n)))
define clean_mains_recipe
clean-mains-$(n):
	@rm -f $(MAINS_$(n))
endef
$(eval $(clean_mains_recipe))
