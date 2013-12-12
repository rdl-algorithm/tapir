n = problems

include mk/template.mk

CHILD_MODULES := rocksample tag

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo
