n = shared

include .make/template.mk

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo
