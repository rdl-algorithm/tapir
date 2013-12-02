n = problems
CHILD_MODULES := RockSample Tag UnderwaterNavModif

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo