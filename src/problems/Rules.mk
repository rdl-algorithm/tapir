n = problems

include mk/template.mk

CHILD_MODULES := RockSample Tag

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo
