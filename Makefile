
# ----------------------------------------------------------------------
# Customizations
# ----------------------------------------------------------------------

#
# root directory of the package
#
ROOT := .

#
# directories
#
SRCDIR = $(ROOT)/src
OBJDIR = $(ROOT)/obj
DEPDIR = $(ROOT)/dep

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
INCDIRS =

#---------------------------------------------------------------------#
# Linker flags
#---------------------------------------------------------------------#

LIBPATH = -L/usr/lib/x86_64-linux-gnu/

LDFLAGS = -g -w -lboost_program_options

# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

SRCS = $(wildcard $(SRCDIR)/*.cc)
HDRS = $(wildcard $(SRCDIR)/*.h)
OBJS = $(patsubst $(SRCDIR)/%.cc,$(OBJDIR)/%.o, $(SRCS))
DEPS = $(patsubst $(SRCDIR)/%.cc,$(DEPDIR)/%.dep, $(SRCS))


# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------
all: CXXFLAGS += $(RELEASE_FLAGS)
all: $(OBJS)

debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: $(OBJS)

$(OBJS) :| $(OBJDIR) $(DEPDIR)
$(DEPDIR) $(OBJDIR) :; mkdir -p $@

$(OBJDIR)/%.o : $(SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) -MM -MD -MT $@ -c $< -o $@
	@mv $(OBJDIR)/$*.d $(DEPDIR)/$*.dep

clean:
	rm -rf $(OBJDIR)
	rm -rf $(DEPDIR)

-include $(DEPS)

.PHONY: all clean debug

# DO NOT DELETE
