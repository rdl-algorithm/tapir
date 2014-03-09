MODULE_NAME = nav2d

ifdef HAS_ROOT_MAKEFILE

TARGET_NAMES := solve simulate
EXTRA_LINKER_DEPS_nav2d := $$(OBJS_geometry)
include .make/problem-template.mk

else
ROOT = ../../..
include $(ROOT)/.make/redirect.mk
endif
