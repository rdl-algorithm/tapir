MODULE_NAME = shared

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS := geometry
include .make/template.mk
include .make/build.mk

else
ROOT = ../../..
include $(ROOT)/.make/redirect.mk
endif
