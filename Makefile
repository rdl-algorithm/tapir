# Root project folder
ROOT := .

# Build configurations
CFGS := release debug
DEFAULT_CFG := release

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------

CC   := gcc
CXX  := g++

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------
override INCDIRS     += -I$(ROOT)/src

override CPPFLAGS    += $(INCDIRS)
CPPFLAGS_release     := $(CPPFLAGS)
CPPFLAGS_debug       := $(CPPFLAGS) -DDEBUG

CXXFLAGS_BASE        := -std=c++11
WARN                 :=
override CXXFLAGS    += $(CXXFLAGS_BASE) $(WARN)
ifeq ($(CXX), g++)
  override CXXFLAGS  += -frounding-math
  WARN               += -Wall -Wextra -Weffc++
else
  WARN               += -Weverything -Wno-c++98-compat
endif
CXXFLAGS_release     := $(CXXFLAGS) -O3
CXXFLAGS_debug       := $(CXXFLAGS) -O0 -ggdb

# ----------------------------------------------------------------------
# Linker flags
# ----------------------------------------------------------------------

# Library directories
override LIBDIRS  += -L/usr/lib/x86_64-linux-gnu/

override LDLIBS  += -lboost_program_options
override LDFLAGS += $(LIBDIRS)
LDFLAGS_debug     = $(LDFLAGS) -g
LDFLAGS_release   = $(LDFLAGS)

# ----------------------------------------------------------------------
# Configuration of code cleaners.
# ----------------------------------------------------------------------

BEAUTIFY_EXCLUDES  = ./src/problems/ProgramOptions.hpp
BEAUTIFY_EXCLUDES += ./src/problems/RockSample/RockSampleOptions.hpp
BEAUTIFY_EXCLUDES += ./src/problems/Tag/TagOptions.hpp
BEAUTIFY_EXCLUDES += ./src/problems/UnderwaterNavModif/UnderwaterNavModifOptions.hpp
BEAUTIFY_CFG = $(ROOT)/mk/uncrustify.cfg
BEAUTIFY_CMD = uncrustify -c $(BEAUTIFY_CFG)
BEAUTIFY_FLAGS = --no-backup

IWYU_MAPPING_FILE = $(ROOT)/mk/mappings.imp
IWYU_CMD = include-what-you-use
IWYU_FLAGS = -Xiwyu --mapping_file=$(IWYU_MAPPING_FILE) -Xiwyu --verbose=3
IWYU_FLAGS += $(CPPFLAGS) $(CXXFLAGS_BASE)

IWYU_FIX_CMD = fix-includes
IWYU_FIX_FLAGS  = --separate_c_cxx
IWYU_FIX_FLAGS += --nosafe_headers --comments $(INCDIRS)
IWYU_FIX_FLAGS += -o $(dir $@)

# ----------------------------------------------------------------------
# General-purpose recipes
# ----------------------------------------------------------------------
#
define recipe_template
COMPILE_RECIPE_$(1) = $$(CXX) $$(CPPFLAGS_$(1)) $$(CXXFLAGS_$(1)) -MMD -c $$< -o $$@
LINK_RECIPE_$(1)    = $$(CXX) $$(LDFLAGS_$(1)) $$^ $$(LDLIBS) -o $$@
endef
$(foreach cfg,$(CFGS),$(eval $(call recipe_template,$(cfg))))

MKDIR_RECIPE = mkdir -p $@

# Code cleaning recipes.
BEAUTIFY_RECIPE = $(BEAUTIFY_CMD) $(BEAUTIFY_FLAGS) $<
IWYU_RECIPE = $(IWYU_CMD) $(IWYU_FLAGS) $< 2>&1 | tee $@
IWYU_FIX_RECIPE = $(IWYU_FIX_CMD) $(IWYU_FIX_FLAGS) < $< 2>&1 | tee $@
IWYU_FORCE_RECIPE = echo "\#include \"../EMPTY_HEADER.hpp\"" >> $<
IWYU_DOFIX_RECIPE = cp -p $(dir $@)/$* $<

# ----------------------------------------------------------------------
# Universal grouping targets
# ----------------------------------------------------------------------

.PHONY: default nothing all build clean
.PHONY: build-all clean-all
default: build-$(DEFAULT_CFG)-solver ;
all build build-all: build-$(DEFAULT_CFG)-all ;
clean: clean-all ;

.PHONY: beautify-all iwyu-all iwyu-fix-all iwyu-force-all iwyu-dofix-all iwyu-clean-all
.PHONY: beautify iwyu iwyu-fix iwyu-force iwyu-dofix iwyu-clean

beautify: beautify-all ;
iwyu: iwyu-all ;
iwyu-fix: iwyu-fix-all ;
iwyu-force: iwyu-force-all ;
iwyu-dofix: iwyu-dofix-all ;
iwyu-clean: iwyu-clean-all ;

clean-all:
	rm -rf builds iwyu-out
iwyu-clean-all:
	rm -rf iwyu-out

define phonies_template
.PHONY: $(1) build-$(1) clean-$(1)
.PHONY: $(1)-all build-$(1)-all clean-$(1)-all
$(1) build-$(1) $(1)-all: build-$(1)-all ;
clean-$(1): clean-$(1)-all ;
clean-$(1)-all:
	rm -rf builds/$(1)
endef
$(foreach cfg,$(CFGS),$(eval $(call phonies_template,$(cfg))))

# Start including other makefiles.
dir := $(ROOT)/src
include mk/stack.mk
