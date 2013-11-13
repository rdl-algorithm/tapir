
# ----------------------------------------------------------------------
# Customizations
# ----------------------------------------------------------------------

#
# root directory of the package
#
ROOT = .

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

#CXXFLAGS = -DNO_FREETYPE `freetype-config --cflags`
CXXFLAGS = -std=c++11 -DDISTL1 $(INCDIRS)
DEBUG_FLAGS = -DDEBUG -g -O0
RELEASE_FLAGS = -frounding-math -O3

INCDIRS = -I$(SRCDIR)

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
PL_DEPS  = $(patsubst $(PL_SRCDIR)/%.cc,$(PL_DEPDIR)/%.d, $(PL_SRCS))

PR_SRCS = $(PR_SRCDIR)/RockSampleModel.cc
PR_HDRS = $(PR_SRCDIR)/RockSampleModel.h $(PR_SRCDIR)/options.h
PR_OBJS  = $(patsubst $(PR_SRCDIR)/%.cc,$(PR_OBJDIR)/%.o, $(PR_SRCS))
PR_DEPS  = $(patsubst $(PR_SRCDIR)/%.cc,$(PR_DEPDIR)/%.d, $(PR_SRCS))

TARGET_SOLVER = $(DESTDIR)/solve
SRC_SOLVER = $(PR_SRCDIR)/solve.cc
OBJ_SOLVER = $(PR_OBJDIR)/solve.o

TARGET_SIMULATOR = $(DESTDIR)/simulate
SRC_SIMULATOR = $(PR_SRCDIR)/simulate.cc
OBJ_SIMULATOR = $(PR_OBJDIR)/simulate.o

SRCS = $(PL_SRCS) $(PR_SRCS)
HDRS = $(PL_HDRS) $(PR_HDRS)
OBJS = $(PL_OBJS) $(PR_OBJS)
DEPS = $(PL_DEPS) $(PR_DEPS) $(PR_DEPDIR)/solve.d $(PR_OBJDIR)/simulate.d

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

$(PL_OBJS) :| $(PL_OBJDIR)
$(PL_OBJDIR) :; mkdir $(PL_OBJDIR)
$(PL_DEPS) :| $(PL_DEPDIR)
$(PL_DEPDIR) :; mkdir $(PL_DEPDIR)
$(PR_OBJS) :| $(PR_OBJDIR)
$(PR_OBJDIR) :; mkdir $(PR_OBJDIR)
$(PR_DEPS) :| $(PR_DEPDIR)
$(PR_DEPDIR) :; mkdir $(PR_DEPDIR)

# Compile and generate dependency info
# 1. Compile the .cpp file
# 2. Generate dependency information, explicitly specifying the target name
# 3. The final three lines do a little bit of sed magic. The following
#    sub-items all correspond to the single sed command below:
#    a. sed: Strip the target (everything before the colon)
#    b. sed: Remove any continuation backslashes
#    c. fmt -1: List words one per line
#    d. sed: Strip leading spaces
#    e. sed: Add trailing colons
$(PL_OBJDIR)/%.o : $(PL_SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) $(INCDIRS) -c $< -o $@
	$(CXX) -MM -MT $(PL_OBJDIR)/$*.o $(CXXFLAGS) $(INCDIRS) \
		$(PL_SRCDIR)/$*.cc > $(PL_DEPDIR)/$*.d
	@cp -f $(PL_DEPDIR)/$*.d $(PL_DEPDIR)/$*.d.tmp
	@sed -e 's/.*://' -e 's/\\\\$$//' < $(PL_DEPDIR)/$*.d.tmp | fmt -1 | \
		sed -e 's/^ *//' -e 's/$$/:/' >> $(PL_DEPDIR)/$*.d
	@rm -f $(PL_DEPDIR)/$*.d.tmp

$(PR_OBJDIR)/%.o : $(PR_SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) $(INCDIRS) -c $< -o $@
	$(CXX) -MM -MT $(PR_OBJDIR)/$*.o $(CXXFLAGS) $(INCDIRS) \
		$(PR_SRCDIR)/$*.cc > $(PR_DEPDIR)/$*.d
	@cp -f $(PR_DEPDIR)/$*.d $(PR_DEPDIR)/$*.d.tmp
	@sed -e 's/.*://' -e 's/\\\\$$//' < $(PR_DEPDIR)/$*.d.tmp | fmt -1 | \
		sed -e 's/^ *//' -e 's/$$/:/' >> $(PR_DEPDIR)/$*.d
	@rm -f $(PR_DEPDIR)/$*.d.tmp

.PHONY: clean
clean:
	rm -f $(OBJS) $(OBJ_SOLVER) $(OBJ_SIMULATOR)
	rm -f $(TARGET_SOLVER) $(TARGET_SIMULATOR)
	rm -fd $(PL_OBJDIR) $(PR_OBJDIR)

.PHONY: test
test:
	echo $(PR_SRCS)
	echo $(PR_HDRS)
	echo $(PR_OBJS)
	echo $(PR_DEPS)

-include $(DEPS)

# DO NOT DELETE
