n = tag
TARGET_NAMES_$(n) := solve simulate
include .make/problem-template.mk

%-$(n): $(d)/%
	@echo
$(d)/%: %
	@echo
