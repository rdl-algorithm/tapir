
# ----------------------------------------------------------------------
# Customizations
# ----------------------------------------------------------------------

# Directory for this problem
PROBLEM := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# Directories
SRCDIR = $(PROBLEM)/src

OUTDIR_RELEASE = $(PROBLEM)/release
BINDIR_RELEASE = $(OUTDIR_RELEASE)/bin
OBJDIR_RELEASE = $(OUTDIR_RELEASE)/obj
DIRS_RELEASE = $(BINDIR_RELEASE) $(OBJDIR_RELEASE)

OUTDIR_DEBUG = $(PROBLEM)/debug
BINDIR_DEBUG = $(OUTDIR_DEBUG)/bin
OBJDIR_DEBUG = $(OUTDIR_DEBUG)/obj
DIRS_DEBUG = $(BINDIR_DEBUG) $(OBJDIR_DEBUG)
DIRS_DEBUG = $(BINDIR_DEBUG) $(OBJDIR_DEBUG)

# ----------------------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------------------

INCDIRS += -I$(SRCDIR)

# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

MAINS := $(PROBLEM)/solve $(PROBLEM)/simulate
SRCS := $(wildcard $(SRCDIR)/*.cpp)
HDRS := $(wildcard $(SRCDIR)/*.hpp)

MAIN_SRCS := $(patsubst $(PROBLEM)/%, $(SRCDIR)/%.cpp, $(MAINS))
PROBLEM_SRCS := $(filter-out $(MAIN_SRCS), $(SRCS))

BINS_RELEASE :=      $(patsubst $(PROBLEM)/%, $(BINDIR_RELEASE)/%,   $(MAINS))
MAIN_OBJS_RELEASE := $(patsubst $(PROBLEM)/%, $(OBJDIR_RELEASE)/%.o, $(MAINS))
MAIN_DEPS_RELEASE := $(patsubst $(PROBLEM)/%, $(OBJDIR_RELEASE)/%.d, $(MAINS))
PROBLEM_OBJS_RELEASE := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR_RELEASE)/%.o, $(PROBLEM_SRCS))
PROBLEM_DEPS_RELEASE := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR_RELEASE)/%.d, $(PROBLEM_SRCS))
OBJS_RELEASE := $(MAIN_OBJS_RELEASE) $(PROBLEM_OBJS_RELEASE)
DEPS_RELEASE := $(MAIN_DEPS_RELEASE) $(PROBLEM_DEPS_RELEASE)

BINS_DEBUG :=      $(patsubst $(PROBLEM)/%, $(BINDIR_DEBUG)/%,   $(MAINS))
MAIN_OBJS_DEBUG := $(patsubst $(PROBLEM)/%, $(OBJDIR_DEBUG)/%.o, $(MAINS))
MAIN_DEPS_DEBUG := $(patsubst $(PROBLEM)/%, $(OBJDIR_DEBUG)/%.d, $(MAINS))
PROBLEM_OBJS_DEBUG := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR_DEBUG)/%.o, $(PROBLEM_SRCS))
PROBLEM_DEPS_DEBUG := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR_DEBUG)/%.d, $(PROBLEM_SRCS))
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
clean: clean-release clean-debug clean-iwyu

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

$(BINDIR_RELEASE)/%: $(OBJDIR_RELEASE)/%.o $(SOLVER_OBJS_RELEASE) $(PROBLEM_OBJS_RELEASE)
	$(LINK_RECIPE)
$(BINDIR_DEBUG)/%: $(OBJDIR_DEBUG)/%.o $(SOLVER_OBJS_DEBUG) $(PROBLEM_OBJS_DEBUG)
	$(LINK_RECIPE)

$(OBJDIR_RELEASE)/%.o $(OBJDIR_DEBUG)/%.o: $(SRCDIR)/%.cpp
	$(COMPILE_RECIPE)

# Rules for making directories
$(BINS_RELEASE): | $(BINDIR_RELEASE)
$(OBJS_RELEASE): | $(OBJDIR_RELEASE)
$(DIRS_RELEASE): | $(OUTDIR_RELEASE)
$(BINS_DEBUG): | $(BINDIR_DEBUG)
$(OBJS_DEBUG): | $(OBJDIR_DEBUG)
$(DIRS_DEBUG): | $(OUTDIR_DEBUG)
$(DIRS_RELEASE) $(DIRS_DEBUG) $(OUTDIR_RELEASE) $(OUTDIR_DEBUG):
	$(MKDIR_RECIPE)

# Inclusion of automatic dependencies
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
IWYU_FIX := $(addsuffix .FIXED, $(IWYU))

.PHONY: iwyu-fix
iwyu-fix: $(IWYU_FIX)

.PHONY: iwyu
iwyu: $(IWYU)

.PHONY: clean-iwyu
clean-iwyu:
	rm -f $(IWYU) $(IWYU_FIX)

$(IWYU_PAIRED) : $(DIR_IWYU)/%.cpp.iwyu: $(SRCDIR)/%.cpp $(SRCDIR)/%.hpp $(IWYU_MAPPING_FILE)
	$(IWYU_RECIPE)
$(IWYU_LONE) : $(DIR_IWYU)/%pp.iwyu: $(SRCDIR)/%pp $(IWYU_MAPPING_FILE)
	$(IWYU_RECIPE)
$(IWYU_FIX): %.FIXED : %
	$(IWYU_FIX_RECIPE)

# Folder structure
$(IWYU) : | $(DIR_IWYU)
$(DIR_IWYU):
	$(MKDIR_RECIPE)

# DO NOT DELETE
