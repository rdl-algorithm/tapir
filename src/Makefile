MODULE_NAME = src

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS := solver problems
include .make/template.mk

else
ROOT = ..
include $(ROOT)/.make/redirect.mk
endif
