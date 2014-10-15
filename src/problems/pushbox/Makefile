MODULE_NAME = pushbox
TARGET_NAMES := solve simulate

ifdef HAS_ROOT_MAKEFILE

include .make/problem-template.mk

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS): call-upwards ;
call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
