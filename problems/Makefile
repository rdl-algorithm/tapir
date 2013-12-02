# Edit Rules.mk to change settings for this subdirectory, or
# edit the root Makefile to change overall settings.
.PHONY: all $(MAKECMDGOALS) call-top
all: MAKECMDGOALS += build

all $(MAKECMDGOALS): call-top
call-top:
	@make -C .. $(addsuffix -problems,$(MAKECMDGOALS))
