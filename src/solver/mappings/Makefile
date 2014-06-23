MODULE_NAME = mappings

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS = actions observations

include .make/template.mk
OBJS_mappings = $(OBJS_action_mappings) $(OBJS_observation_mappings)
include .make/build.mk

else
ROOT = ../../..
include $(ROOT)/.make/redirect.mk
endif
