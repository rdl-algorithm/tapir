MODULE_NAME = solver

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS = indexing mappings serialization abstract-problem changes
include .make/template.mk

# Elements that make up the library
define library_template
LIB_solver_$(1) := $(BUILDDIR_solver_$(1))/libabt.a
OUTPUTS_TO_CLEAN_solver_$(1) += $$(LIB_solver_$(1))
MEMBERS_solver_$(1) := $$$$(OBJS_solver_$(1))
MEMBERS_solver_$(1) += $$$$(OBJS_global_$(1))
OUTPUTS_TO_CLEAN_solver_$(1) += $$(OBJS_global_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call library_template,$(cfg))))

define child_loop_template
MEMBERS_solver_$(1) += $$$$(OBJS_$(2)_$(1))
OUTPUTS_TO_CLEAN_solver_$(1) += $$(OBJS_$(2)_$(1))
endef
$(foreach cfg,$(CFGS),$(foreach folder,$(CHILD_FOLDERS),$(eval $(call child_loop_template,$(cfg),$(folder)))))

$(info $(MEMBERS_solver_debug))

# Include a configuration for building just the object files
define make_archive_template
build-$(1)-solver: $(LIB_solver_$(1))
$(LIB_solver_$(1)): $$(MEMBERS_solver_$(1))
	$(AR) rcs $$@ $(MEMBERS_solver_$(1))

# Rebuild the archive if this Makefile changes.
$(BUILDDIR_solver_$(1))/libabt.a: $(d)/Makefile
endef
$(foreach cfg,$(CFGS),$(eval $(call make_archive_template,$(cfg))))

else
ROOT = ../..
include $(ROOT)/.make/redirect.mk
endif
