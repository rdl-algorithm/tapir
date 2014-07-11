sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)

# Default: name=directory; no children.
MODULE_NAME := $(d)
CHILD_DIRS :=

include $(d)/Makefile
NAME_$(d) := $(MODULE_NAME)
CHILD_DIRS_$(MODULE_NAME) := $(CHILD_DIRS)
PATH_$(MODULE_NAME) := $(d)
ALL_OBJS_$(MODULE_NAME) := $(OBJS_$(MODULE_NAME))

define child_template
dir := $(d)/$(1)
include .make/stack.mk
endef
$(foreach directory,$(CHILD_DIRS),$(eval $(call child_template,$(directory))))

define child_objs_template
ALL_OBJS_$(NAME_$(d)) += $(ALL_OBJS_$(NAME_$(d)/$(1)))
endef
$(foreach directory,$(CHILD_DIRS_$(NAME_$(d))),$(eval $(call child_objs_template,$(directory))))

d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
