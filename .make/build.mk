# Include a configuration for building just the object files
define build_template
build-$(1)-$(MODULE_NAME): $(OBJS_$(MODULE_NAME)_$(1))
endef
$(foreach cfg,$(CFGS),$(eval $(call build_template,$(cfg))))
