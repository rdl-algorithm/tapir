dir := $(dir $(lastword $(MAKEFILE_LIST)))

.PHONY: $(MAKECMDGOALS) call-top
TARGETS := $(addsuffix -$(MODULE_NAME),$(MAKECMDGOALS))

default: TARGETS += build-$(MODULE_NAME)

default $(MAKECMDGOALS): call-top
call-top:
	@$(MAKE) -C $(ROOT) $(TARGETS) REDIRECT=$(MODULE_NAME)
