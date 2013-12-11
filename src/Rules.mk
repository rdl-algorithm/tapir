n = src

include mk/template.mk

CHILD_MODULES := solver problems

$(d)/%: %
	@echo
