ROOT := .
ABS_ROOT := $(abspath $(ROOT))
HAS_ROOT_MAKEFILE := true

# ----------------------------------------------------------------------
# General configuration settings
# ----------------------------------------------------------------------
# Eigen is optional, so you don't really need it unless you want to use
# policy iteration to solve an MDP
# (see src/problems/shared/policy_iteration.hpp).
HAS_EIGEN            := false

# ----------------------------------------------------------------------
# ROS configuration settings
# ----------------------------------------------------------------------
# Custom source directory for BOOST 1.48 => required for Ubuntu 12.04!
# If you have Boost installed normally, just leave this blank.
CUSTOM_BOOST_148_DIR :=
# Main ROS setup script
ROS_SCRIPT           := /opt/ros/hydro/setup.sh
# Directory for the Catkin workspace
CATKIN_WS_DIR        := $(ABS_ROOT)/../catkin_ws
# Directory in which to find V-REP
VREP_DIR             := $(ABS_ROOT)/../vrep

# ----------------------------------------------------------------------
# Basic build settings
# ----------------------------------------------------------------------

# Default build configuration
DEFAULT_CFG := release
ifndef CFG
  CFG := $(DEFAULT_CFG)
endif

# Default build target.
DEFAULT_TARGET := all

# Output directories.
BUILDDIR := $(ROOT)/builds/$(CFG)
PROBLEMS_DIR := $(ROOT)/problems

# You can put local settings in local.make
-include local.make

# Use the shell to expand user home directories.
CUSTOM_BOOST_148_DIR := $(shell echo $(CUSTOM_BOOST_148_DIR))
ROS_SCRIPT := $(shell echo $(ROS_SCRIPT))
CATKIN_WS_DIR := $(shell echo $(CATKIN_WS_DIR))
VREP_DIR := $(shell echo $(VREP_DIR))

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
override INCDIRS     += -I$(ROOT)/include -I$(ROOT)/src -I$(ROOT)/src/options

override CPPFLAGS    += -DROOT_PATH=$(ABS_ROOT) $(INCDIRS)
ifeq ($(HAS_EIGEN),true)
  override CPPFLAGS  += -DHAS_EIGEN
endif
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
  CWARN               += -pedantic -Wall -Wextra -Wshadow
  CWARN               += -Wswitch-default -fmax-errors=5
  CXXWARN             := $(CWARN) -Weffc++
  override CXXFLAGS   += -frounding-math

# For GCC >= 4.9 we can use C++1y and color diagnostics
  GCC_VER_GTE49 := $(shell echo `gcc -dumpversion | cut -f1-2 -d.` \>= 4.9 | bc )
  ifeq ($(GCC_VER_GTE49),1)
    CXXFLAGS_BASE     := -std=c++1y
    override LDFLAGS  += -fdiagnostics-color=auto
    override CXXFLAGS += -fdiagnostics-color=auto
    AR                := gcc-ar
  endif
endif

# Configuration-specific flags
ifeq ($(CFG),release)
  override CXXFLAGS  += -O3 -flto
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
override LIBDIRS += -L/usr/lib/x86_64-linux-gnu/
override LDFLAGS += $(LIBDIRS) -flto -O3 -fuse-linker-plugin

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

ALL_DERIVED_DIRS = $(ROOT)/builds $(DOC_OUTPUT_DIRS)

# clean-all is a straight-up "rm -rf" - it doesn't check for "junk files".
.PHONY: clean-all
clean-all:
	@echo Removing all derived directories!
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

dir := $(ROOT)/docs
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

# ----------------------------------------------------------------------
# ROS catkin_make system
# ----------------------------------------------------------------------
# Recipe to generate a script to run ROS
define ROS_SCRIPT_RECIPE
	@echo "#!/bin/sh" > $@
	@printf 'env TAPIR_DIR="$(ABS_ROOT)" ' >> $@
	@printf 'TAPIR_WS_DIR="$(CATKIN_WS_DIR)" ' >> $@
	@printf 'VREP_DIR="$(VREP_DIR)" ' >> $@
	@printf '"$(ABS_ROOT)/.ros-scripts/run.sh" $(1)\n' >> $@
	@chmod +x $@
endef

# Catkin workspace directories
CATKIN_SRC_DIR := $(CATKIN_WS_DIR)/src
ROS_ABT_DIR := $(CATKIN_SRC_DIR)/tapir
$(ROS_ABT_DIR):| $(CATKIN_SRC_DIR)
	cd $(CATKIN_SRC_DIR); ln -s $(ABS_ROOT) tapir
$(CATKIN_SRC_DIR):
	$(MKDIR_RECIPE)

# Tag script
TAG_SCRIPT := $(ROOT)/problems/tag/simulate-ros
$(TAG_SCRIPT): Makefile
	$(call ROS_SCRIPT_RECIPE,tag)

# Environment variables to pass to catkin_make
CATKIN_MAKE_ENVS :=
ifeq ($(HAS_EIGEN),true)
	CATKIN_MAKE_ENVS += HAS_EIGEN=1
endif

# Phony target to rebuild the V-REP plugin
.PHONY: vrep_plugin
vrep_plugin:
	@env ROS_SCRIPT=$(ROS_SCRIPT) VREP_DIR=$(VREP_DIR) .ros-scripts/build_vrep_plugin.sh

# Phony target for Boost, if a custom directory is used.
.PHONY: boost
ifeq ($(CUSTOM_BOOST_148_DIR),)
# No custom Boost path => assume everything is OK
else

# Custom boost path => inform catkin_make of the custom path.
CATKIN_MAKE_ENVS += TAPIR_BOOST_148=$(CUSTOM_BOOST_148_DIR)

BOOST_TEST = $(CUSTOM_BOOST_148_DIR)/include/boost/config.hpp

# Recipe to automatically download and install boost:
boost: | $(BOOST_TEST)
$(BOOST_TEST) :
	$(ABS_ROOT)/.ros-scripts/get_boost_148.sh
	$(ABS_ROOT)/.ros-scripts/patch_boost_148.sh
	env TAPIR_BOOST_148=$(CUSTOM_BOOST_148_DIR) $(ABS_ROOT)/.ros-scripts/build_boost_148.sh
endif

CATKIN_MAKE := env $(CATKIN_MAKE_ENVS) catkin_make

.PHONY: indigo-ws
indigo-ws: $(CATKIN_SRC_DIR)
	@cat $(ROOT)/.ros-scripts/.rosinstall > $(CATKIN_SRC_DIR)/.rosinstall
	@cd $(CATKIN_SRC_DIR) && wstool update

.PHONY: ros-scripts ros
ros-scripts: $(TAG_SCRIPT)
ros: boost ros-scripts | $(ROS_ABT_DIR)
	. $(ROS_SCRIPT) && cd $(CATKIN_WS_DIR) && $(CATKIN_MAKE)

.PHONY: clean-ros-scripts clean-ros
clean-ros-scripts:
	@rm -f $(TAG_SCRIPT)
clean-ros: clean-ros-scripts
	. $(ROS_SCRIPT) && cd $(CATKIN_WS_DIR) && $(CATKIN_MAKE) clean

