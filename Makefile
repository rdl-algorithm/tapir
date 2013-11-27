.PHONY: release
default: release
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
.PHONY: release
release: CPPFLAGS += $(CPPFLAGS_RELEASE)
release: CXXFLAGS += $(CXXFLAGS_RELEASE)
release: $(SOLVER_OBJS_RELEASE)

.PHONY: debug
debug: CPPFLAGS += $(CPPFLAGS_DEBUG)
debug: CXXFLAGS += $(CXXFLAGS_DEBUG)
debug: $(SOLVER_OBJS_DEBUG)

-include $(SOLVER_DEPS_RELEASE)
-include $(SOLVER_DEPS_DEBUG)

.PHONY: clean-release
clean-release:
	rm -f $(SOLVER_OBJS_RELEASE) $(SOLVER_DEPS_RELEASE)

.PHONY: clean-debug
clean-debug:
	rm -f $(SOLVER_OBJS_DEBUG) $(SOLVER_DEPS_DEBUG)

.PHONY: clean
clean: clean-release clean-debug

.PHONY: rmdirs
rmdirs: clean
	rm -fd $(SOLVER_DIRS_RELEASE) $(SOLVER_DIRS_DEBUG)
	rm -fd $(SOLVER_OUTDIR_RELEASE) $(SOLVER_OUTDIR_DEBUG)
	rm -rf $(ROOT)/problems/*/release $(ROOT)/problems/*/debug

.PHONY: redo
redo:
	$(MAKE) rmdirs
	$(MAKE)

# DO NOT DELETE
