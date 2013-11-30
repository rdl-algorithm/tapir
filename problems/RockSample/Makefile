
# ----------------------------------------------------------------------
# Customizations
# ----------------------------------------------------------------------

# Directory for this problem
PROBLEM := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# Directories
SRCDIR = $(PROBLEM)/src
OUTDIR_RELEASE = $(PROBLEM)/release
DIRS_RELEASE = $(OUTDIR_RELEASE)/bin $(OUTDIR_RELEASE)/obj
OUTDIR_DEBUG = $(PROBLEM)/debug
DIRS_DEBUG = $(OUTDIR_DEBUG)/bin $(OUTDIR_DEBUG)/obj

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------

CC = gcc
CXX = g++
LINKER = g++

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------

INCDIRS += -I$(SRCDIR)

CPPFLAGS = -DDISTL1 $(INCDIRS)
CPPFLAGS_RELEASE =
CPPFLAGS_DEBUG = -DDEBUG

CXXFLAGS = -Wall -Wextra -Weffc++ -std=c++11
CXXFLAGS_RELEASE = -frounding-math -O3
CXXFLAGS_DEBUG = -g -O0

# ----------------------------------------------------------------------
# Linker flags
# ----------------------------------------------------------------------

LIBDIRS += -L/usr/lib/x86_64-linux-gnu/
LDLIBS += -lboost_program_options

LDFLAGS = $(LIBDIRS)
LDFLAGS_DEBUG = -g
LDFLAGS_RELEASE =

# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

MAINS := $(PROBLEM)/solve $(PROBLEM)/simulate
SRCS := $(wildcard $(SRCDIR)/*.cpp)
HDRS := $(wildcard $(SRCDIR)/*.hpp)

MAIN_SRCS := $(patsubst $(PROBLEM)/%, $(SRCDIR)/%.cpp, $(MAINS))
PROBLEM_SRCS := $(filter-out $(MAIN_SRCS), $(SRCS))

BINS_RELEASE :=      $(patsubst $(PROBLEM)/%, $(OUTDIR_RELEASE)/bin/%,   $(MAINS))
MAIN_OBJS_RELEASE := $(patsubst $(PROBLEM)/%, $(OUTDIR_RELEASE)/obj/%.o, $(MAINS))
MAIN_DEPS_RELEASE := $(patsubst $(PROBLEM)/%, $(OUTDIR_RELEASE)/obj/%.d, $(MAINS))
PROBLEM_OBJS_RELEASE := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_RELEASE)/obj/%.o, $(PROBLEM_SRCS))
PROBLEM_DEPS_RELEASE := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_RELEASE)/obj/%.d, $(PROBLEM_SRCS))
OBJS_RELEASE := $(MAIN_OBJS_RELEASE) $(PROBLEM_OBJS_RELEASE)
DEPS_RELEASE := $(MAIN_DEPS_RELEASE) $(PROBLEM_DEPS_RELEASE)

BINS_DEBUG :=      $(patsubst $(PROBLEM)/%, $(OUTDIR_DEBUG)/bin/%,   $(MAINS))
MAIN_OBJS_DEBUG := $(patsubst $(PROBLEM)/%, $(OUTDIR_DEBUG)/obj/%.o, $(MAINS))
MAIN_DEPS_DEBUG := $(patsubst $(PROBLEM)/%, $(OUTDIR_DEBUG)/obj/%.d, $(MAINS))
PROBLEM_OBJS_DEBUG := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_DEBUG)/obj/%.o, $(PROBLEM_SRCS))
PROBLEM_DEPS_DEBUG := $(patsubst $(SRCDIR)/%.cpp, $(OUTDIR_DEBUG)/obj/%.d, $(PROBLEM_SRCS))
OBJS_DEBUG := $(MAIN_OBJS_DEBUG) $(PROBLEM_OBJS_DEBUG)
DEPS_DEBUG := $(MAIN_DEPS_DEBUG) $(PROBLEM_DEPS_DEBUG)

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------

.PHONY: default
default: all
# Defines paths & targets for the solver
include $(PROBLEM)/../../Makefile.common

.PHONY: all
all: release

.PHONY: clean
clean: clean-release clean-debug

.PHONY: release
release: CPPFLAGS += $(CPPFLAGS_RELEASE)
release: CXXFLAGS += $(CXXFLAGS_RELEASE)
release: LDFLAGS += $(LDFLAGS_RELEASE)
release: $(BINS_RELEASE)
	cp $(BINS_RELEASE) $(PROBLEM)

.PHONY: clean-release
clean-release:
	rm -f $(MAINS) $(BINS_RELEASE)
	rm -f $(OBJS_RELEASE) $(DEPS_RELEASE)
	rm -f $(SOLVER_OBJS_RELEASE) $(SOLVER_DEPS_RELEASE)

.PHONY: debug
debug: CPPFLAGS += $(CPPFLAGS_DEBUG)
debug: CXXFLAGS += $(CXXFLAGS_DEBUG)
debug: LDFLAGS += $(LDFLAGS_DEBUG)
debug: $(BINS_DEBUG)
	cp $(BINS_DEBUG) $(PROBLEM)

.PHONY: clean-debug
clean-debug:
	rm -f $(MAINS) $(BINS_DEBUG)
	rm -f $(OBJS_DEBUG) $(DEPS_DEBUG)
	rm -f $(SOLVER_OBJS_RELEASE) $(SOLVER_DEPS_RELEASE)

$(OUTDIR_RELEASE)/bin/%: $(OUTDIR_RELEASE)/obj/%.o $(SOLVER_OBJS_RELEASE) $(PROBLEM_OBJS_RELEASE)
	$(LINKER) $(LDFLAGS) $^ $(LDLIBS) -o $@
$(OUTDIR_DEBUG)/bin/%: $(OUTDIR_DEBUG)/obj/%.o $(SOLVER_OBJS_DEBUG) $(PROBLEM_OBJS_DEBUG)
	$(LINKER) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BINS_RELEASE): | $(OUTDIR_RELEASE)/bin
$(OBJS_RELEASE): | $(OUTDIR_RELEASE)/obj
$(DIRS_RELEASE): | $(OUTDIR_RELEASE)
$(BINS_DEBUG): | $(OUTDIR_DEBUG)/bin
$(OBJS_DEBUG): | $(OUTDIR_DEBUG)/obj
$(DIRS_DEBUG): | $(OUTDIR_DEBUG)
$(DIRS_RELEASE) $(DIRS_DEBUG) $(OUTDIR_RELEASE) $(OUTDIR_DEBUG):
	mkdir -p $@

$(OUTDIR_RELEASE)/obj/%.o $(OUTDIR_DEBUG)/obj/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MD -c $< -o $@

-include $(MAIN_DEPS_RELEASE) $(PROBLEM_DEPS_RELEASE) $(SOLVER_DEPS_RELEASE)
-include $(MAIN_DEPS_DEBUG) $(PROBLEM_DEPS_DEBUG) $(SOLVER_DEPS_DEBUG)

# ----------------------------------------------------------------------
# Automatic management of #include directives
# ----------------------------------------------------------------------

DIR_IWYU := $(PROBLEM)/iwyu
HDRS_PAIRED := $(filter $(HDRS), $(patsubst %.cpp, %.hpp, $(SRCS)))
SRCS_PAIRED := $(patsubst %.hpp, %.cpp, $(HDRS_PAIRED))
SRCS_LONE := $(filter-out $(SRCS_PAIRED), $(SRCS))
HDRS_LONE := $(filter-out $(HDRS_PAIRED), $(HDRS))
IWYU_PAIRED := $(patsubst $(SRCDIR)/%.cpp, $(DIR_IWYU)/%.cpp.iwyu, $(SRCS_PAIRED))
IWYU_LONE := $(patsubst $(SRCDIR)/%pp, $(DIR_IWYU)/%pp.iwyu, $(SRCS_LONE) $(HDRS_LONE))
IWYU := $(IWYU_PAIRED) $(IWYU_LONE)
IWYU_FIX := $(addsuffix .FIX, $(IWYU))

.PHONY: iwyu-fix $(IWYU_FIX)
iwyu-fix: $(IWYU_FIX)

.PHONY: iwyu
iwyu: $(IWYU)

.PHONY: clean-iwyu
clean-iwyu:
	rm -f $(IWYU)

$(IWYU) : | $(DIR_IWYU)
$(DIR_IWYU):
	mkdir -p $@

$(IWYU_PAIRED) : $(DIR_IWYU)/%.cpp.iwyu: $(SRCDIR)/%.cpp $(SRCDIR)/%.hpp
	include-what-you-use -Xiwyu --verbose=3 $(CPPFLAGS) $(CXXFLAGS) $< > $@ 2>&1 || exit 0
$(IWYU_LONE) : $(DIR_IWYU)/%pp.iwyu: $(SRCDIR)/%pp
	include-what-you-use -Xiwyu --verbose=3 $(CPPFLAGS) $(CXXFLAGS) $< > $@ 2>&1 || exit 0

$(IWYU_FIX): %.FIX : %
	fix-includes --nosafe_headers --comments $(INCDIRS) < $< || exit 0

# DO NOT DELETE
