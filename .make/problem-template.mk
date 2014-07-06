include .make/template.mk

TARGET_NAMES_$(n) := $(TARGET_NAMES)

BINDIR_$(n)       := $(BINDIR)/basic/$(n)
_BIN_BUILD_$(n)   := $(OBJDIR_$(n))/%
_BIN_OUTPUT_$(n)  := $(BINDIR_$(n))/%

# ----------------------------------------------------------------------
# Directories, file patterns, and directory dependencies
# ----------------------------------------------------------------------
TGTS_BUILD_$(n)       := $(patsubst %,$(_BIN_BUILD_$(n)),$(TARGET_NAMES_$(n)))
TGTS_OUTPUT_$(n)      := $(patsubst %,$(_BIN_OUTPUT_$(n)),$(TARGET_NAMES_$(n)))
TGTS_SRCDIR_$(n)      := $(patsubst %,$(SRCDIR_$(n))/%,$(TARGET_NAMES_$(n)))
OUTPUTS_TO_CLEAN_$(n) += $(TGTS_BUILD_$(n)) $(TGTS_OUTPUT_$(n)) $(TGTS_SRCDIR_$(n))

OBJS_TGT_$(n)         := $(patsubst $(_BIN_BUILD_$(n)),$(_O_$(n)),$(TGTS_$(n)))
OBJS_NTGT_$(n)        := $(filter-out $(OBJS_TGT_$(n)),$(OBJS_$(n)))

# Directory prerequsites
$(TGTS_BUILD_$(n)): | $(OBJDIR_$(n))
$(TGTS_OUTPUT_$(n)): | $(BINDIR_$(n))

$(BINDIR_$(n)):
	$(MKDIR_RECIPE)

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
LINKER_ARGS_$(n) += -Lbuilds/shared -Wl;-rpath=$(ABS_ROOT)/builds/shared -ltapir
else
LINKER_ARGS_$(n) += $$(LIB_solver)
endif

LINKER_ARGS_$(n) += -lspatialindex
LINKER_ARGS_$(n) += $(EXTRA_LINKER_ARGS_$(n))

# This is so we can escape commas!
,=,
# Linking rule for the executable targets.
define problem_build_template
$(TGTS_BUILD_$(n)): $(_BIN_BUILD_$(n)): $(_O_$(n)) $$(LINKER_DEPS_$(n))
	$$(subst ;,$$(,),$$(call LINK_RECIPE,$$< $(LINKER_ARGS_$(n))))
endef
$(eval $(problem_build_template))

# Executables will be copied to the "bin" folder for convenience
build-$(n): $(TGTS_OUTPUT_$(n)) $(TGTS_SRCDIR_$(n))

.PHONY: $(TGTS_OUTPUT_$(n)) $(TGTS_SRCDIR_$(n))
$(TGTS_OUTPUT_$(n)): $(_BIN_OUTPUT_$(n)): $(_BIN_BUILD_$(n))
	cp $< $@

$(TGTS_SRCDIR_$(n)): $(SRCDIR_$(n))/%: $(_BIN_BUILD_$(n))
	cp $< $@
