MODULE_NAME = problems
CHILD_FOLDERS := shared rocksample tag homecare

ifdef HAS_ROOT_MAKEFILE

include .make/template.mk

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS) call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
