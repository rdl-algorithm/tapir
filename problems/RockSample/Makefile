
# ----------------------------------------------------------------------
# Customizations
#----------------------------------------------------------------------

# Directory for this problem
PROBLEM := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
.PHONY: default
default: release
include $(PROBLEM)/../../Makefile.common

# Directories
SRCDIR = $(PROBLEM)/src
OUTDIR_RELEASE = $(PROBLEM)/release
DIRS_RELEASE = $(OUTDIR_RELEASE)/bin $(OUTDIR_RELEASE)/obj $(OUTDIR_RELEASE)/dep
OUTDIR_DEBUG = $(PROBLEM)/debug
DIRS_DEBUG = $(OUTDIR_DEBUG)/bin $(OUTDIR_DEBUG)/obj $(OUTDIR_DEBUG)/dep

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------
CC = gcc
CXX = g++
LINKER = g++

#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#

INCDIRS += -I$(SRCDIR)

CPPFLAGS = -DDISTL1 $(INCDIRS)
CPPFLAGS_RELEASE =
CPPFLAGS_DEBUG = -DDEBUG

CXXFLAGS = -Wall -Wextra -Weffc++ -std=c++11
CXXFLAGS_RELEASE = -frounding-math -O3
CXXFLAGS_DEBUG = -g -O0

#---------------------------------------------------------------------#
# Linker flags
#---------------------------------------------------------------------#

LIBDIRS += -L/usr/lib/x86_64-linux-gnu/
LDLIBS += -lboost_program_options

LDFLAGS = $(LIBDIRS)
LDFLAGS_DEBUG = -g
LDFLAGS_RELEASE =

# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

MAINS := $(PROBLEM)/solve $(PROBLEM)/simulate
MAIN_SRCS := $(patsubst $(PROBLEM)/%, $(SRCDIR)/%.cpp, $(MAINS))
PROBLEM_SRCS := $(filter-out $(MAIN_SRCS), $(wildcard $(SRCDIR)/*.cpp))

BINS_RELEASE :=      $(patsubst $(PROBLEM)/%, $(OUTDIR_RELEASE)/bin/%,   $(MAINS))
MAIN_OBJS_RELEASE := $(patsubst $(PROBLEM)/%, $(OUTDIR_RELEASE)/obj/%.o, $(MAINS))
MAIN_DEPS_RELEASE := $(patsubst $(PROBLEM)/%, $(OUTDIR_RELEASE)/dep/%.d, $(MAINS))
PROBLEM_OBJS_RELEASE := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_RELEASE)/obj/%.o, $(PROBLEM_SRCS))
PROBLEM_DEPS_RELEASE := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_RELEASE)/dep/%.d, $(PROBLEM_SRCS))
OBJS_RELEASE := $(MAIN_OBJS_RELEASE) $(PROBLEM_OBJS_RELEASE)
DEPS_RELEASE := $(MAIN_DEPS_RELEASE) $(PROBLEM_DEPS_RELEASE)

BINS_DEBUG :=      $(patsubst $(PROBLEM)/%, $(OUTDIR_DEBUG)/bin/%,   $(MAINS))
MAIN_OBJS_DEBUG := $(patsubst $(PROBLEM)/%, $(OUTDIR_DEBUG)/obj/%.o, $(MAINS))
MAIN_DEPS_DEBUG := $(patsubst $(PROBLEM)/%, $(OUTDIR_DEBUG)/dep/%.d, $(MAINS))
PROBLEM_OBJS_DEBUG := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_DEBUG)/obj/%.o, $(PROBLEM_SRCS))
PROBLEM_DEPS_DEBUG := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_DEBUG)/dep/%.d, $(PROBLEM_SRCS))
OBJS_DEBUG := $(MAIN_OBJS_DEBUG) $(PROBLEM_OBJS_DEBUG)
DEPS_DEBUG := $(MAIN_DEPS_DEBUG) $(PROBLEM_DEPS_DEBUG)

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------
#
.PHONY: release
release: CPPFLAGS += $(CPPFLAGS_RELEASE)
release: CXXFLAGS += $(CXXFLAGS_RELEASE)
release: LDFLAGS += $(LDFLAGS_RELEASE)
release: $(BINS_RELEASE)
	cp $(BINS_RELEASE) $(PROBLEM)

.PHONY: debug
debug: CPPFLAGS += $(CPPFLAGS_DEBUG)
debug: CXXFLAGS += $(CXXFLAGS_DEBUG)
debug: LDFLAGS += $(LDFLAGS_DEBUG)
debug: $(BINS_DEBUG)
	cp $(BINS_DEBUG) $(PROBLEM)

$(OUTDIR_RELEASE)/bin/%: $(OUTDIR_RELEASE)/obj/%.o $(SOLVER_OBJS_RELEASE) $(PROBLEM_OBJS_RELEASE)
	$(LINKER) $(LDFLAGS) $^ $(LDLIBS) -o $@
$(OUTDIR_DEBUG)/bin/%: $(OUTDIR_DEBUG)/obj/%.o $(SOLVER_OBJS_DEBUG) $(PROBLEM_OBJS_DEBUG)
	$(LINKER) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BINS_RELEASE): | $(OUTDIR_RELEASE)/bin
$(OBJS_RELEASE): | $(OUTDIR_RELEASE)/obj $(OUTDIR_RELEASE)/dep
$(DIRS_RELEASE): | $(OUTDIR_RELEASE)
$(BINS_DEBUG): | $(OUTDIR_DEBUG)/bin
$(OBJS_DEBUG): | $(OUTDIR_DEBUG)/obj $(OUTDIR_DEBUG)/dep
$(DIRS_DEBUG): | $(OUTDIR_DEBUG)
$(DIRS_RELEASE) $(DIRS_DEBUG) $(OUTDIR_RELEASE) $(OUTDIR_DEBUG):
	mkdir -p $@

$(OUTDIR_RELEASE)/obj/%.o $(OUTDIR_DEBUG)/obj/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MD -c $< -o $@
	@mv $(basename $@).d $(dir $@)/../dep/$*.d

.PHONY: clean-release
clean-release:
	rm -f $(MAINS) $(BINS_RELEASE)
	rm -f $(OBJS_RELEASE) $(DEPS_RELEASE)
	rm -f $(SOLVER_OBJS_RELEASE) $(SOLVER_DEPS_RELEASE)

.PHONY: clean-debug
clean-debug:
	rm -f $(MAINS) $(BINS_DEBUG)
	rm -f $(OBJS_DEBUG) $(DEPS_DEBUG)
	rm -f $(SOLVER_OBJS_DEBUG) $(SOLVER_DEPS_DEBUG)

.PHONY: clean
clean: clean-release clean-debug

.PHONY: rmdirs
rmdirs: clean
	rm -fd $(DIRS_RELEASE) $(DIRS_DEBUG)
	rm -fd $(OUTDIR_RELEASE) $(OUTDIR_DEBUG)
	rm -fd $(SOLVER_OBJDIR) $(SOLVER_DEPDIR)

.PHONY: redo
redo:
	$(MAKE) rmdirs
	$(MAKE)

-include $(MAIN_DEPS_RELEASE) $(PROBLEM_DEPS_RELEASE) $(SOLVER_DEPS_RELEASE)
-include $(MAIN_DEPS_DEBUG) $(PROBLEM_DEPS_DEBUG) $(SOLVER_DEPS_DEBUG)

# DO NOT DELETE
