MODULE_NAME = solver

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS = indexing mappings serialization abstract-problem changes search
include .make/template.mk

# Elements that make up the library
LIB_solver               := $(BUILDDIR_solver)/libabt.a
OUTPUTS_TO_CLEAN_solver  += $(LIB_solver)
MEMBERS_solver            = $(OBJS_solver)
MEMBERS_solver           += $(OBJS_global)

define child_loop_template
MEMBERS_solver          += $$(OBJS_$(1))
clean-solver: clean-$(1)
endef
$(foreach folder,$(CHILD_FOLDERS),$(eval $(call child_loop_template,$(folder))))

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
