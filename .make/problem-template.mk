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
# Problem-specific library
# ----------------------------------------------------------------------
# Library pattern.
LIB_$(n) := $(BUILDDIR_$(n))/lib$(n).a
OUTPUTS_TO_CLEAN_$(n) += $(LIB_$(n))
MEMBERS_$(n) := $(OBJS_NTGT_$(n)) $(OBJS_shared) $(OBJS_options) $(OBJS_inih)

define library_template
$(LIB_$(n)): $$(MEMBERS_$(n))
	$(AR) rcs $$@ $(MEMBERS_$(n))
endef
$(eval $(library_template))

# Rebuild the library if this template changes
$(LIB_$(n)): .make/problem-template.mk

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------
# Dependencies for the linker
LINKER_DEPS_$(n) := $(LIB_$(n)) $$(LIB_solver)


LINKER_ARGS_$(n) += $(LIB_$(n))
ifeq ($(CFG),shared)
LINKER_ARGS_$(n) += -Lbuilds/shared -Wl;-rpath=$(ABS_ROOT)/builds/shared -labt
else
LINKER_ARGS_$(n) += $$(LIB_solver)
endif

LINKER_ARGS_$(n) += -lspatialindex
LINKER_ARGS_$(n) += $(EXTRA_LINKER_ARGS_$(n))

# This is so we can escape commas!
,=,
# Linking rule for the executable targets.
define problem_build_template
$(TGTS_$(n)): $(_BIN_$(n)): $(_O_$(n)) $$(LINKER_DEPS_$(n))
	$$(subst ;,$$(,),$$(call LINK_RECIPE,$$< $(LINKER_ARGS_$(n))))

# Executables will be copied to the main folder for convenience
build-$(n): $$(TGTS_$(n))
	cp $$^ $(d)
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
