sp              := $(sp).x
dirstack_$(sp)  := $(d)
namestack_$(sp) := $(n)
d               := $(dir)

CHILD_MODULES :=
include $(d)/Rules.mk

define child_template
dir := $(d)/$(1)
include mk/stack.mk
endef
$(foreach module,$(CHILD_MODULES),$(eval $(call child_template,$(module))))

d               := $(dirstack_$(sp))
n               := $(namestack_$(sp))
sp              := $(basename $(sp))
