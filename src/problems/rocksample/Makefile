# Edit Rules.mk to change settings for this subdirectory, or
# edit the root Makefile to change overall settings.
.PHONY: build $(MAKECMDGOALS) call-top
build: MAKECMDGOALS += build

build $(MAKECMDGOALS): call-top
call-top:
	@$(MAKE) -C ../../.. $(addsuffix -rocksample,$(MAKECMDGOALS))
