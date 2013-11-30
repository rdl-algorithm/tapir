# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------

CC = gcc
CXX = g++
LINKER = g++

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------

CPPFLAGS = -DDISTL1 $(INCDIRS)
CPPFLAGS_RELEASE =
CPPFLAGS_DEBUG = -DDEBUG

CXXFLAGS = -Wall -Wextra -Weffc++ -std=c++11
CXXFLAGS_RELEASE = -frounding-math -O3
CXXFLAGS_DEBUG = -g -O0

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

.PHONY: default
default: release
# Defines paths & targets for the solver.
include Makefile.common

.PHONY: all
all: release
	$(MAKE) -C $(ROOT)/problems/Tag release
	$(MAKE) -C $(ROOT)/problems/RockSample release
	$(MAKE) -C $(ROOT)/problems/UnderwaterNavModif release

.PHONY: clean
clean: clean-release clean-debug clean-iwyu
	$(MAKE) -C $(ROOT)/problems/Tag clean
	$(MAKE) -C $(ROOT)/problems/RockSample clean
	$(MAKE) -C $(ROOT)/problems/UnderwaterNavModif clean

.PHONY: release
release: CPPFLAGS += $(CPPFLAGS_RELEASE)
release: CXXFLAGS += $(CXXFLAGS_RELEASE)
release: $(SOLVER_OBJS_RELEASE)
-include $(SOLVER_DEPS_RELEASE)

.PHONY: clean-release
clean-release:
	rm -f $(SOLVER_OBJS_RELEASE) $(SOLVER_DEPS_RELEASE)

.PHONY: debug
debug: CPPFLAGS += $(CPPFLAGS_DEBUG)
debug: CXXFLAGS += $(CXXFLAGS_DEBUG)
debug: $(SOLVER_OBJS_DEBUG)
-include $(SOLVER_DEPS_DEBUG)

.PHONY: clean-debug
clean-debug:
	rm -f $(SOLVER_OBJS_DEBUG) $(SOLVER_DEPS_DEBUG)


# ----------------------------------------------------------------------
# Automatic management of #include directives
# ----------------------------------------------------------------------

SOLVER_DIR_IWYU := $(ROOT)/iwyu
SOLVER_HDRS_PAIRED := $(filter $(SOLVER_HDRS), $(patsubst %.cpp, %.hpp, $(SOLVER_SRCS)))
SOLVER_SRCS_PAIRED := $(patsubst %.hpp, %.cpp, $(SOLVER_HDRS_PAIRED))
SOLVER_SRCS_LONE := $(filter-out $(SOLVER_SRCS_PAIRED), $(SOLVER_SRCS))
SOLVER_HDRS_LONE := $(filter-out $(SOLVER_HDRS_PAIRED), $(SOLVER_HDRS))

SOLVER_IWYU_PAIRED := $(patsubst $(SOLVER_SRCDIR)/%.cpp, $(SOLVER_DIR_IWYU)/%.cpp.iwyu, $(SOLVER_SRCS_PAIRED))
SOLVER_IWYU_LONE := $(patsubst $(SOLVER_SRCDIR)/%pp, $(SOLVER_DIR_IWYU)/%pp.iwyu, $(SOLVER_SRCS_LONE) $(SOLVER_HDRS_LONE))
SOLVER_IWYU := $(SOLVER_IWYU_PAIRED) $(SOLVER_IWYU_LONE)
SOLVER_IWYU_FIX := $(addsuffix .FIX, $(SOLVER_IWYU))

.PHONY: iwyu-fix-all
iwyu-fix-all: iwyu-fix
	$(MAKE) -C $(ROOT)/problems/Tag iwyu-fix
	$(MAKE) -C $(ROOT)/problems/RockSample iwyu-fix
	$(MAKE) -C $(ROOT)/problems/UnderwaterNavModif iwyu-fix

.PHONY: iwyu-fix $(SOLVER_IWYU_FIX)
iwyu-fix: $(SOLVER_IWYU_FIX)

.PHONY: iwyu
iwyu: $(SOLVER_IWYU)

.PHONY: clean-iwyu
clean-iwyu:
	rm -f $(SOLVER_IWYU)

$(SOLVER_IWYU) : | $(SOLVER_DIR_IWYU)
$(SOLVER_DIR_IWYU):
	mkdir -p $@

$(SOLVER_IWYU_PAIRED) : $(SOLVER_DIR_IWYU)/%.cpp.iwyu: $(SOLVER_SRCDIR)/%.cpp $(SOLVER_SRCDIR)/%.hpp
	include-what-you-use -Xiwyu --verbose=3 $(CPPFLAGS) $(CXXFLAGS) $< > $@ 2>&1 || exit 0
$(SOLVER_IWYU_LONE) : $(SOLVER_DIR_IWYU)/%pp.iwyu: $(SOLVER_SRCDIR)/%pp
	include-what-you-use -Xiwyu --verbose=3 $(CPPFLAGS) $(CXXFLAGS) $< > $@ 2>&1 || exit 0

$(SOLVER_IWYU_FIX) : %.FIX : %
	fix-includes --nosafe_headers --comments $(INCDIRS) < $< || exit 0

# DO NOT DELETE
