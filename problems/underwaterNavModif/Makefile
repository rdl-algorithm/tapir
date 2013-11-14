
# ----------------------------------------------------------------------
# Customizations
# ----------------------------------------------------------------------

#
# root directory of the package
#
ROOT = ../..
PROBLEM = .

#
# directories
#
DESTDIR = .
PL_SRCDIR = $(ROOT)/src
PL_OBJDIR = $(ROOT)/obj
PR_SRCDIR = $(PROBLEM)
PR_OBJDIR = $(PROBLEM)/obj
DEPDIR = $(PROBLEM)/dep


# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------
CC = gcc
CXX = g++
LINKER = g++

#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#

#CXXFLAGS = -DNO_FREETYPE `freetype-config --cflags`
CXXFLAGS = -std=c++11 -DDISTL1 $(INCDIRS)
DEBUG_FLAGS = -DDEBUG -g -O0
RELEASE_FLAGS = -frounding-math -O3

INCDIRS = -I$(PL_SRCDIR) -I$(PR_SRCDIR)

#---------------------------------------------------------------------#
# Linker flags
#---------------------------------------------------------------------#

LIBPATH = -L/usr/lib/x86_64-linux-gnu/

LDFLAGS = -g -w -lboost_program_options

# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

PL_SRCS = $(wildcard $(PL_SRCDIR)/*.cc)
PL_HDRS = $(wildcard $(PL_SRCDIR)/*.h)
PL_OBJS  = $(patsubst $(PL_SRCDIR)/%.cc,$(PL_OBJDIR)/%.o, $(PL_SRCS))
PL_DEPS  = $(patsubst $(PL_SRCDIR)/%.cc,$(DEPDIR)/%.dep, $(PL_SRCS))

PR_SRCS = $(PR_SRCDIR)/UnderwaterNavModifModel.cc $(PR_SRCDIR)/StRoadmap.cc
PR_HDRS = $(PR_SRCDIR)/UnderwaterNavModifModel.h  $(PR_SCRDIR)/StRoadmmap.h\
		  $(PR_SRCDIR)/options.h
PR_OBJS  = $(patsubst $(PR_SRCDIR)/%.cc,$(PR_OBJDIR)/%.o, $(PR_SRCS))
PR_DEPS  = $(patsubst $(PR_SRCDIR)/%.cc,$(DEPDIR)/%.dep, $(PR_SRCS))

TARGET_SOLVER = $(DESTDIR)/solve
SRC_SOLVER = $(PR_SRCDIR)/solve.cc
OBJ_SOLVER = $(PR_OBJDIR)/solve.o

TARGET_SIMULATOR = $(DESTDIR)/simulate
SRC_SIMULATOR = $(PR_SRCDIR)/simulate.cc
OBJ_SIMULATOR = $(PR_OBJDIR)/simulate.o

SRCS = $(PL_SRCS) $(PR_SRCS)
HDRS = $(PL_HDRS) $(PR_HDRS)
OBJS = $(PL_OBJS) $(PR_OBJS)
DEPS = $(PL_DEPS) $(PR_DEPS) $(DEPDIR)/solve.dep $(DEPDIR)/simulate.dep

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------
.PHONY: all
all: CXXFLAGS += $(RELEASE_FLAGS)
all: $(TARGET_SOLVER) $(TARGET_SIMULATOR)

.PHONY: debug
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: $(TARGET_SOLVER) $(TARGET_SIMULATOR)

$(TARGET_SOLVER): $(OBJS) $(OBJ_SOLVER)
	$(LINKER) $(LIBPATH) $(OBJS) $(OBJ_SOLVER) $(LDFLAGS) -o $@

$(TARGET_SIMULATOR): $(OBJS) $(OBJ_SIMULATOR)
	$(LINKER) $(LIBPATH) $(OBJS) $(OBJ_SIMULATOR) $(LDFLAGS) -o $@

$(PL_OBJS) :| $(PL_OBJDIR) $(DEPDIR)
$(PR_OBJS) :| $(PR_OBJDIR) $(DEPDIR)
$(DEPDIR) $(PL_OBJDIR) $(PR_OBJDIR) :; mkdir -p $@

$(PL_OBJDIR)/%.o : $(PL_SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) -MD -MT $@ -c $< -o $@
	@mv $(PL_OBJDIR)/$*.d $(DEPDIR)/$*.dep

$(PR_OBJDIR)/%.o : $(PR_SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) -MD -MT $@ -c $< -o $@
	@mv $(PR_OBJDIR)/$*.d $(DEPDIR)/$*.dep

.PHONY: clean
clean:
	rm -f $(TARGET_SOLVER) $(TARGET_SIMULATOR)
	rm -rf $(PL_OBJDIR) $(PR_OBJDIR) $(DEPDIR)

-include $(DEPS)

# DO NOT DELETE
