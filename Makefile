# Root project folder
ROOT := .
ABS_ROOT := $(abspath $(ROOT))

HAS_ROOT_MAKEFILE := true

# Defaul build configuration
DEFAULT_CFG := release
ifndef CFG
  CFG := $(DEFAULT_CFG)
endif

# Default build target.
DEFAULT_TARGET := solver

BUILDDIR := $(ROOT)/builds/$(CFG)
BINDIR   := $(ROOT)/bin

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------

AR   := ar
CC   := gcc
CXX  := g++

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------
# Preprocessor flags.
override INCDIRS     += -I$(ROOT)/src -I$(ROOT)/src/options

override CPPFLAGS    += -DROOT_PATH=$(ABS_ROOT) $(INCDIRS)
ifeq ($(CFG),debug)
  override CPPFLAGS  += -DDEBUG
endif

# Compile flags
CXXFLAGS_BASE        := -std=c++11
CXXWARN              :=
CWARN                :=
override CXXFLAGS    += $(CXXFLAGS_BASE) $(CXXWARN)
override CFLAGS      += $(CWARN)

# Differences in flags between clang++ and g++
ifeq ($(CXX),clang++)
  override INCDIRS    += -I/usr/include/c++/4.8
#  CWARN               += -Weverything
  CWARN               += -Wno-c++98-compat
  CXXWARN             := $(CWARN)
else ifneq (,$(findstring g++,$(CXX)))
  CWARN               += -Wpedantic -Wall -Wextra -Wshadow
  CWARN               += -Wswitch-default -Wfatal-errors
  CXXWARN             := $(CWARN) -Weffc++
  override CXXFLAGS   += -frounding-math

# For GCC >= 4.9 we can use C++1y and color diagnostics
  GCC_VERSION := $(shell expr `$(CXX) -dumpversion`)
  ifeq ($(GCC_VERSION), 4.9)
	CXXFLAGS_BASE     := -std=c++1y
	override LDFLAGS  += -fdiagnostics-color=auto
	override CXXFLAGS += -fdiagnostics-color=auto
  endif
endif

# Configuration-specific flags
ifeq ($(CFG),release)
  override CXXFLAGS  += -O3
else ifeq ($(CFG),shared)
  override CXXFLAGS  += -O3 -fpic
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
# Redirection handling.
# ----------------------------------------------------------------------
ifdef REDIRECT
ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS := $(REDIRECT)
endif
# Try a straight-up redirect.
.PHONY: $(MAKECMDGOALS) call-root
$(MAKECMDGOALS): call-root ;
call-root:
	@$(MAKE) --no-print-directory $(MAKECMDGOALS) REDIRECT= REDIRECTED_FROM=$(REDIRECT)
else
# If we're not redirecting, everything works as per normal...

.PHONY: default
default: $(DEFAULT_TARGET) ;


# ----------------------------------------------------------------------
# General-purpose recipes
# ----------------------------------------------------------------------
# The reciple for .cpp files - called via $(call arg)
CXX_RECIPE     = $(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -c $(1) -o $@
# The recipe for .c files - called via $(call arg)
CC_RECIPE      = $(CC) $(CPPFLAGS) $(CFLAGS) -MMD -c $(1) -o $@
# The recipe for .o files - called via $(call arg)
LINK_RECIPE    = $(CXX) $(LDFLAGS) $(1) $(LDLIBS) -o $@

# The recipe for directories; doesn't need to be $(call)-ed.
MKDIR_RECIPE   = @mkdir -p $@

# ----------------------------------------------------------------------
# Documentation targets
# ----------------------------------------------------------------------

# Directory dependency
docs/generated/README.md docs/generated/BUILD.md: docs/generated
docs/generated:
	$(MKDIR_RECIPE)

# Adapt the README.md so it works nicely with Doxygen.
docs/generated/README.md: README.md docs/doxygen_links.md Makefile
	@echo "# ABT Documentation {#mainpage}\n" > $@
	@cat README.md docs/doxygen_links.md >> $@
	@perl -0pi -e 's/^(.*)(\n=+)$$/\1 {#abt}\2/mg' $@
	@perl -0pi -e 's/^(.*)(\n-+)$$/\1 {#\L\1}\2/mg' $@
	@perl -0pi -e 's/^([#]{2,}\s+)(.*)$$/\1\2 {#\L\2}/mg' $@
	@perl -pi -e 's/({#.*})/($$res = $$1) =~ s\/ \/-\/g,$$res/eg' $@

# Do the same thing for the build system README
docs/generated/BUILD.md: .make/README.md docs/doxygen_links.md Makefile
	@cat .make/README.md docs/doxygen_links.md > $@

# Now a command to generate the documentation.
.PHONY: doc
doc: docs/Doxyfile docs/generated/README.md docs/generated/BUILD.md
	doxygen docs/Doxyfile

.PHONY: clean-doc
clean-doc:
	@echo Removing documentation folders!
	@rm -rfv html/ docs/generated | grep "directory" ; true

# ----------------------------------------------------------------------
# Universal grouping targets
# ----------------------------------------------------------------------
.PHONY: nothing
nothing: ;

# build-all is a special target - every sub-target should add itself as
# a prerequisite for build-all
.PHONY: build-all

# Include some shorthand targets for "build-all" and "build-solver"
.PHONY: all build
all: build-all
build: build-solver

# A list of all the directories for generated output.
ALL_DERIVED_DIRS := $(ROOT)/builds $(ROOT)/bin $(ROOT)/html $(ROOT)/docs/generated

# clean-all is a straight-up "rm -rf" - it doesn't check for "junk files".
.PHONY: clean-all
clean-all:
	@echo Removing all derived folders!
	@rm -rfv $(ALL_DERIVED_DIRS) | grep "directory" ; true

# clean only cleans actual targets.
.PHONY: clean

# Some stuff used for code beautification
include .make/beautify-settings.mk

# Turn on secondary expansion for cross-referencing!
.SECONDEXPANSION:

# Start including other makefiles.
dir := $(ROOT)/src
include .make/stack.mk


# After we're done
ifdef REDIRECTED_FROM
$(info Redirected from: $(PATH_$(REDIRECTED_FROM)))
endif

# Print out our build configuration
ifeq ($(MAKECMDGOALS),)
$(info Building $(DEFAULT_TARGET))
else
$(info Building $(MAKECMDGOALS))
endif
$(info CFG=$(CFG))
$(info )

endif
