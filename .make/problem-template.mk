include .make/template.mk

# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------
define problem_defaults
TGTS_$(n)_$(1)        := $(patsubst %,$(_BIN_$(n)_$(1)),$(TARGET_NAMES_$(n)))
ALL_OUTPUTS_$(n)_$(1) += $$(TGTS_$(n)_$(1))
OBJS_TGT_$(n)_$(1)    := $$(patsubst $(_BIN_$(n)_$(1)),$(_O_$(n)_$(1)),$$(TGTS_$(n)_$(1)))
OBJS_NTGT_$(n)_$(1)   := $$(filter-out $$(OBJS_TGT_$(n)_$(1)),$(OBJS_$(n)_$(1)))

# Directory prerequsites
$$(TGTS_$(n)_$(1)): | $$(BINDIR_$(n)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call problem_defaults,$(cfg))))

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Linking rule for the executable targets.
define problem_build_template
$(TGTS_$(n)_$(1)): $(_BIN_$(n)_$(1)): $(_O_$(n)_$(1)) $(OBJS_NTGT_$(n)_$(1)) $(OBJS_solver_$(1))
	$$(LINK_RECIPE_$(1))
# Executables will be copied to the main folder for convenience
build-$(1)-$(n): $(TGTS_$(n)_$(1))
	cp $$^ $(d)
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
