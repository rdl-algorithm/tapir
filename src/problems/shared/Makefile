MODULE_NAME = shared
CHILD_FOLDERS := geometry

ifdef HAS_ROOT_MAKEFILE

include .make/template.mk
include .make/build.mk

ifneq ($(HAS_EIGEN),true)
OBJS_shared := $(filter-out $(BUILDDIR_shared)/policy_iteration.o,$(OBJS_shared))
endif

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS): call-upwards ;
call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
