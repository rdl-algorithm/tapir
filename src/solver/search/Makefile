MODULE_NAME = search
CHILD_FOLDERS := steppers action-choosers

ifdef HAS_ROOT_MAKEFILE

include .make/template.mk
include .make/build.mk   

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS) call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
