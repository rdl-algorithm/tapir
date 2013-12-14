MODULE_NAME = solver

ifdef HAS_ROOT_MAKEFILE

include .make/template.mk

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Build the solver by building all the object files.
define build_template
build-$(1)-$(MODULE_NAME): $(OBJS_$(MODULE_NAME)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call build_template,$(cfg))))

else
ROOT = ../..
include $(ROOT)/.make/redirect.mk
endif
