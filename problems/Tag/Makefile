
# ----------------------------------------------------------------------
# Customizations
#----------------------------------------------------------------------

# Directory for this problem
PROBLEM := $(dir $(lastword $(MAKEFILE_LIST)))
default: all
include $(PROBLEM)/../../Makefile.common

#
# directories
#
PROBLEM_SRCDIR = $(PROBLEM)/src
PROBLEM_OBJDIR = $(PROBLEM)/obj
PROBLEM_DEPDIR = $(PROBLEM)/dep
DESTDIR = $(PROBLEM)

VPATH = $(SOLVER_SRCDIR):$(PROBLEM_SRCDIR)

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------
CC = gcc
CXX = g++
LINKER = g++

#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#

CXXFLAGS = -std=c++11 -DDISTL1 $(INCDIRS)
DEBUG_FLAGS = -DDEBUG -g -O0
RELEASE_FLAGS = -frounding-math -O3
INCDIRS = -I$(SOLVER_SRCDIR) -I$(PROBLEM_SRCDIR)

#---------------------------------------------------------------------#
# Linker flags
#---------------------------------------------------------------------#

LDFLAGS = -g -w -lboost_program_options -L/usr/lib/x86_64-linux-gnu/

# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

MAIN_SRCS := $(PROBLEM_SRCDIR)/solve.cc $(PROBLEM_SRCDIR)/simulate.cc
MAIN_OBJS := $(patsubst $(PROBLEM_SRCDIR)/%.cc, $(PROBLEM_OBJDIR)/%.o,   $(MAIN_SRCS))
MAIN_DEPS := $(patsubst $(PROBLEM_SRCDIR)/%.cc, $(PROBLEM_DEPDIR)/%.dep, $(MAIN_SRCS))
MAINS     := $(patsubst $(PROBLEM_SRCDIR)/%.cc, $(DESTDIR)/%, $(MAIN_SRCS))

PROBLEM_SRCS := $(filter-out $(MAIN_SRCS), $(wildcard $(PROBLEM_SRCDIR)/*.cc))
PROBLEM_HDRS := $(wildcard $(PROBLEM_SRCDIR)/*.h)
PROBLEM_OBJS := $(patsubst $(PROBLEM_SRCDIR)/%.cc, $(PROBLEM_OBJDIR)/%.o,   $(PROBLEM_SRCS))
PROBLEM_DEPS := $(patsubst $(PROBLEM_SRCDIR)/%.cc, $(PROBLEM_DEPDIR)/%.dep, $(PROBLEM_SRCS))

SRCS := $(SOLVER_SRCS) $(PROBLEM_SRCS) $(MAIN_SRCS)
HDRS := $(SOLVER_HDRS) $(PROBLEM_HDRS)
OBJS := $(SOLVER_OBJS) $(PROBLEM_OBJS) $(MAIN_OBJS)
DEPS := $(SOLVER_DEPS) $(PROBLEM_DEPS) $(MAIN_DEPS)

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------
.SECONDEXPANSION:
.PHONY: all
all: CXXFLAGS += $(RELEASE_FLAGS)
all: $(MAINS)

.PHONY: debug
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: $(MAINS)

$(MAINS): $(PROBLEM_OBJDIR)/$$@.o $(SOLVER_OBJS) $(PROBLEM_OBJS)
	$(LINKER) $^ $(LDFLAGS) -o $@

$(PROBLEM_OBJS) $(MAIN_OBJS): | $(PROBLEM_OBJDIR) $(PROBLEM_DEPDIR)
$(PROBLEM_DEPDIR) $(PROBLEM_OBJDIR):
	mkdir -p $@

$(PROBLEM_OBJDIR)/%.o: $(PROBLEM_SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@
	@mv $(PROBLEM_OBJDIR)/$*.d $(PROBLEM_DEPDIR)/$*.dep

.PHONY: clean
clean:
	rm -f $(MAINS)
	rm -f $(OBJS)
	rm -f $(DEPS)

.PHONY:
rmdirs:
	rm -rf $(PROBLEM_OBJDIR) $(PROBLEM_DEPDIR) $(SOLVER_OBJDIR) $(SOLVER_DEPDIR)

-include $(DEPS)

# DO NOT DELETE
