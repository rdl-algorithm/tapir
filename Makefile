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

CXXFLAGS = -Wall -std=c++11 -DDISTL1 $(INCDIRS)
DEBUG_FLAGS = -DDEBUG -g -O0
RELEASE_FLAGS = -frounding-math -O3
# INCDIRS +=

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------/
.PHONY: all
all: CXXFLAGS += $(RELEASE_FLAGS)
all: $(SOLVER_OBJS)

.PHONY: debug
debug: CXXFLAGS += $(DEBUG_FLAGS)
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
