
# ----------------------------------------------------------------------
# Customizations
#----------------------------------------------------------------------

# Directory for this problem
PROBLEM := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
default: all
include $(PROBLEM)/../../Makefile.common

# Directories
PROBLEM_SRCDIR = $(PROBLEM)/src
PROBLEM_OBJDIR = $(PROBLEM)/obj
PROBLEM_DEPDIR = $(PROBLEM)/dep
DESTDIR = $(PROBLEM)


# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------
CC = gcc
CXX = g++
LINKER = g++

#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#

INCDIRS += -I$(PROBLEM_SRCDIR)

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

MAINS     := $(DESTDIR)/solve $(DESTDIR)/simulate
MAIN_SRCS := $(patsubst $(DESTDIR)/%, $(PROBLEM_SRCDIR)/%.cc,  $(MAINS))
MAIN_OBJS := $(patsubst $(DESTDIR)/%, $(PROBLEM_OBJDIR)/%.o,   $(MAINS))
MAIN_DEPS := $(patsubst $(DESTDIR)/%, $(PROBLEM_DEPDIR)/%.dep, $(MAINS))

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
all: CPPFLAGS += $(CPPFLAGS_RELEASE)
all: CXXFLAGS += $(CXXFLAGS_RELEASE)
all: LDFLAGS += $(LDFLAGS_RELEASE)
all: $(MAINS)

.PHONY: debug
debug: CPPFLAGS += $(CPPFLAGS_DEBUG)
debug: CXXFLAGS += $(CXXFLAGS_DEBUG)
debug: LDFLAGS += $(LDFLAGS_DEBUG)
debug: $(MAINS)

$(MAINS): $(PROBLEM_OBJDIR)/$$(notdir $$@).o $(SOLVER_OBJS) $(PROBLEM_OBJS)
	$(LINKER) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(PROBLEM_OBJS) $(MAIN_OBJS): | $(PROBLEM_OBJDIR) $(PROBLEM_DEPDIR)
$(PROBLEM_DEPDIR) $(PROBLEM_OBJDIR):
	mkdir -p $@

$(PROBLEM_OBJDIR)/%.o: $(PROBLEM_SRCDIR)/%.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MD -c $< -o $@
	@mv $(PROBLEM_OBJDIR)/$*.d $(PROBLEM_DEPDIR)/$*.d

.PHONY: clean
clean:
	rm -f $(MAINS)
	rm -f $(OBJS)
	rm -f $(DEPS)

.PHONY: rmdirs
rmdirs:
	rm -rf $(PROBLEM_OBJDIR) $(PROBLEM_DEPDIR) $(SOLVER_OBJDIR) $(SOLVER_DEPDIR)

.PHONY: redo
redo:
	$(MAKE) rmdirs
	$(MAKE)

-include $(DEPS)

# DO NOT DELETE
