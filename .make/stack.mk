sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)

CHILD_MODULES :=
include $(d)/Rules.mk

define child_template
dir := $(d)/$(1)
include .make/stack.mk
endef
$(foreach module,$(CHILD_MODULES),$(eval $(call child_template,$(module))))

d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
