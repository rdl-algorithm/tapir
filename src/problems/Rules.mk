n = problems

CHILD_MODULES := shared rocksample tag

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo
