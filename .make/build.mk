# Include a configuration for building just the object files
build-$(MODULE_NAME): $$(OBJS_$(MODULE_NAME))
