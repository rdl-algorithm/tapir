default: all
include Makefile.common

# Compiler & linker
# ----------------------------------------------------------------------
CC = gcc
CXX = g++
LINKER = g++

#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#

CPPFLAGS = -DDISTL1 $(INCDIRS)
CPPFLAGS_RELEASE =
CPPFLAGS_DEBUG = -DDEBUG

CXXFLAGS = -Wall -Wextra -Weffc++ -std=c++11
CXXFLAGS_RELEASE = -frounding-math -O3
CXXFLAGS_DEBUG = -g -O0

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------/
.PHONY: all
all: CPPFLAGS += $(CPPFLAGS_RELEASE)
all: CXXFLAGS += $(CXXFLAGS_RELEASE)
all: $(SOLVER_OBJS)

.PHONY: debug
debug: CPPFLAGS += $(CPPFLAGS_DEBUG)
debug: CXXFLAGS += $(CXXFLAGS_DEBUG)
debug: $(SOLVER_OBJS)

.PHONY: clean
clean:
	rm -f $(SOLVER_OBJS) $(SOLVER_DEPS)

-include $(SOLVER_DEPS)

.PHONY: rmdirs
rmdirs:
	rm -rf $(SOLVER_OBJDIR) $(SOLVER_DEPDIR)

.PHONY: redo
redo:
	$(MAKE) rmdirs
	$(MAKE)

# DO NOT DELETE
