sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)

MODULE_NAME :=
CHILD_FOLDERS :=
include $(d)/Makefile
PATH_$(MODULE_NAME) := $(d)

define child_template
dir := $(d)/$(1)
include .make/stack.mk
endef
$(foreach folder,$(CHILD_FOLDERS),$(eval $(call child_template,$(folder))))

d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
