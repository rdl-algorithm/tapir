MODULE_NAME = solver

ifdef HAS_ROOT_MAKEFILE

CHILD_FOLDERS = indexing mappings serialization geometry
include .make/template.mk
include .make/build.mk

else
ROOT = ../..
include $(ROOT)/.make/redirect.mk
endif
