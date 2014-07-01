MODULE_NAME = global

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS := options solver problems
include .make/template.mk
include .make/build.mk

else
ROOT = ..
include $(ROOT)/.make/redirect.mk
endif
