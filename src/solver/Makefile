MODULE_NAME = solver

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS := indexing mappings serialization abstract-problem changes
CHILD_FOLDERS += search belief-q-estimators action-choosers
include .make/template.mk

# Elements that make up the library
LIB_solver               := $(BUILDDIR_solver)/libabt.a
OUTPUTS_TO_CLEAN_solver  += $(LIB_solver)
MEMBERS_solver            = $(ALL_OBJS_solver) $(OBJS_global)

# Include a configuration for building just the object files
build-solver: $(LIB_solver)
$(LIB_solver): $$(MEMBERS_solver)
	$(AR) rcs $@ $(MEMBERS_solver)

# Rebuild the archive if this Makefile changes.
$(LIB_solver): $(d)/Makefile

else
ROOT = ../..
include $(ROOT)/.make/redirect.mk
endif
