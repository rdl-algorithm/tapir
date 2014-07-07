MODULE_NAME = rocksample
TARGET_NAMES := solve simulate

ifdef HAS_ROOT_MAKEFILE

include .make/problem-template.mk

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS) call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
