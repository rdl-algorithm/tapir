# Root project folder
ROOT := .

HAS_ROOT_MAKEFILE = true

# Build configurations
DEFAULT_CFG := release

ifndef CFG
  CFG := $(DEFAULT_CFG)
endif

$(info Configuration: $(CFG))
$(info )

BUILDDIR = $(ROOT)/builds/$(CFG)

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------

AR   := ar
CXX  := g++

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------
# Preprocessor flags.
override INCDIRS     += -I$(ROOT)/src

override CPPFLAGS    += $(INCDIRS)
ifeq ($(CFG),debug)
  override CPPFLAGS  += -DDEBUG
endif

# Compile flags
CXXFLAGS_BASE        := -std=c++11
WARN                 :=
override CXXFLAGS    += $(CXXFLAGS_BASE) $(WARN)

# Differences in flags between clang++ and g++
ifeq ($(CXX),clang++)
  WARN               += -Weverything -Wno-c++98-compat
endif
ifeq ($(CXX),g++)
  WARN               += -Wpedantic -Wall -Wextra -Wshadow -Weffc++
  WARN               += -Wswitch-default -Wfatal-errors
  override CXXFLAGS  += -frounding-math
endif

# Configuration-specific flags
ifeq ($(CFG),release)
  override CXXFLAGS  += -O3
endif
ifeq ($(CFG),perf)
  override CXXFLAGS  += -O3 -ggdb3
endif
ifeq ($(CFG),gprof)
  override CXXFLAGS  += -O3 -pg
  override LDFLAGS   += -pg
endif
ifeq ($(CFG),debug)
  override CXXFLAGS  += -O0 -ggdb3
endif

# ----------------------------------------------------------------------
# Linker flags
# ----------------------------------------------------------------------

# Library directories
override LIBDIRS  += -L/usr/lib/x86_64-linux-gnu/
override LDFLAGS += $(LIBDIRS)

# ----------------------------------------------------------------------
# General-purpose recipes
# ----------------------------------------------------------------------
COMPILE_CMD = $(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -c -o $@
LINK_CMD    = $(CXX) $(LDFLAGS) -o $@
MKDIR_RECIPE = @mkdir -p $@

# ----------------------------------------------------------------------
# Universal grouping targets
# ----------------------------------------------------------------------

.PHONY: default nothing
default: build-solver ;
nothing: ;

.PHONY: all build build-all
all: build-all
build: build-solver

.PHONY: clean clean-all
clean-all:
	@echo Removing all build folders!
	@rm -rfv $(ROOT)/builds | grep "directory" ; true

# Some stuff used for code beautification
include .make/beautify-settings.mk

# Turn on secondary expansion for cross-referencing!
.SECONDEXPANSION:
# Start including other makefiles.
dir := $(ROOT)/src
include .make/stack.mk

# ----------------------------------------------------------------------
# Redirection handling.
# ----------------------------------------------------------------------
ifdef REDIRECT
# If the target wasn't found, check the global targets.
%-$(REDIRECT): % ;

# If the target can't be found, check local targets.
# %-$(REDIRECT): $(PATH_$(REDIRECT))/% ;
# If not, check for a global target.
# $(PATH_$(REDIRECT))/% : % ;
endif
