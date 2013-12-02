n = tag
TARGET_NAMES_$(n) := solve simulate
include $(d)/../template.mk

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo
