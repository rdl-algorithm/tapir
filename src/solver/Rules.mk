n = solver

include mk/template.mk

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

# Build the solver by building all the object files.
define build_template
build-$(1)-$(n): $(OBJS_$(n)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call build_template,$(cfg))))
