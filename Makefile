# Root project folder
ROOT := .

HAS_ROOT_MAKEFILE := true

# Build configurations
DEFAULT_CFG := release

ifndef CFG
  CFG := $(DEFAULT_CFG)
endif

$(info Configuration: $(CFG))
$(info )

BUILDDIR := $(ROOT)/builds/$(CFG)

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------

AR   := ar
CXX  := g++

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------
# Preprocessor flags.
override INCDIRS     += -I$(ROOT)/src -I$(ROOT)/src/options

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
ifneq (,$(findstring g++,$(CXX)))
  WARN               += -Wpedantic -Wall -Wextra -Wshadow -Weffc++
  WARN               += -Wswitch-default -Wfatal-errors
  override CXXFLAGS  += -frounding-math
  GCC_VERSION := $(shell expr `$(CXX) -dumpversion`)
  ifeq ($(GCC_VERSION), 4.9)
	CXXFLAGS_BASE := -std=c++1y
	override LDFLAGS += -fdiagnostics-color=auto
	override CXXFLAGS += -fdiagnostics-color=auto
  endif
endif

# Configuration-specific flags
ifeq ($(CFG),release)
  override CXXFLAGS  += -O3
else ifeq ($(CFG),profile)
  override CPPFLAGS  += -DGOOGLE_PROFILER
  override CXXFLAGS  += -O3 -ggdb3
  override LDFLAGS   += -ggdb3
  override LDLIBS    += /usr/lib/libprofiler.so.0
else ifeq ($(CFG),prof)
  override CXXFLAGS  += -O3 -p -ggdb3
  override LDFLAGS   += -p -ggdb3
else ifeq ($(CFG),gprof)
  override CXXFLAGS  += -O3 -pg -ggdb3
  override LDFLAGS   += -pg -ggdb3
else ifeq ($(CFG),debug)
  override CXXFLAGS  += -O0 -ggdb3
else
  $(error Could not find configuration $(CFG))
endif

# ----------------------------------------------------------------------
# Linker flags
# ----------------------------------------------------------------------

# Library directories
override LIBDIRS += -L/usr/lib/x86_64-linux-gnu/
override LDFLAGS += $(LIBDIRS)

# ----------------------------------------------------------------------
# General-purpose recipes
# ----------------------------------------------------------------------
COMPILE_RECIPE = $(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -c $(1) -o $@
LINK_RECIPE    = $(CXX) $(LDFLAGS) $(1) $(LDLIBS) -o $@
MKDIR_RECIPE   = @mkdir -p $@

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
