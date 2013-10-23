
# ----------------------------------------------------------------------
# Customizations
# ----------------------------------------------------------------------

#
# root directory of the package
#
ROOT	      = ../../

#
# destination directory
#
DESTDIR	      = ./

#
# name of dependency file
#
DEPFILE       = Makefile.dep

# 
# source directories
#
PLANNER			= $(ROOT)/src/
PROBLEM			= $(ROOT)/problems/underwaterNavigation/

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------
CC            = gcc 

CXX           = g++ 

LINKER	      = g++ 
		
#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#
 
CXXFLAGS =  -DNO_FREETYPE `freetype-config --cflags`  -frounding-math -O3 $(INCDIRS)

INCDIRS	= -I$(PLANNER) -I$(PROBLEM) \

#---------------------------------------------------------------------#
# Linker flags
#---------------------------------------------------------------------#

LIBPATH = -L$(PROBLEM) \
						
LDFLAGS = -g -w \
		
#---------------------------------------------------------------------#
# Archiver
#---------------------------------------------------------------------#

ARCHIVER      = ar

ARFLAGS       = -ruv


# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

SRCS_PLANNER 	= 	$(PLANNER)/Solver.cc \
					$(PLANNER)/Model.cc \
					$(PLANNER)/Histories.cc \
					$(PLANNER)/HistorySeq.cc \
					$(PLANNER)/HistoryEntry.cc \
					$(PLANNER)/PolTree.cc \
					$(PLANNER)/PolNode.cc \
					$(PLANNER)/PolActEdge.cc \
					$(PLANNER)/PolObsEdge.cc \
					$(PLANNER)/Belief.cc \
					$(PLANNER)/State.cc \
					$(PLANNER)/Action.cc \
					$(PLANNER)/Observation.cc 

HDRS_PLANNER 	= 	$(PLANNER)/PtrDeclare.h \
					$(PLANNER)/Solver.h \
					$(PLANNER)/Model.h \
					$(PLANNER)/Histories.h \
					$(PLANNER)/HistorySeq.h \
					$(PLANNER)/HistoryEntry.h \
					$(PLANNER)/PolTree.h \
					$(PLANNER)/PolNode.h \
					$(PLANNER)/PolActEdge.h \
					$(PLANNER)/PolObsEdge.h \
					$(PLANNER)/Belief.h \
					$(PLANNER)/State.h \
					$(PLANNER)/Action.h  \
					$(PLANNER)/Observation.h 

OBJS_PLANNER	= 	$(SRCS_PLANNER:$(PLANNER)/%.cc=%.o)

SRCS_PROBLEM	= 	$(PROBLEM)/UnderwaterNavModel.cc 

HDRS_PROBLEM	= 	$(PROBLEM)/UnderwaterNavModel.h 
		
OBJS_PROBLEM	= 	$(SRCS_PPROBLEM:$(PROBLEM)/%.cc=%.o)

TARGET_SOLVER  	= solve

SRCS_SOLVER		= $(PROBLEM)/solve.cc 

OBJ_SOLVER		= solve.o 

TARGET_SIM		= simulate

SRCS_SIMULATOR	= $(PROBLEM)/simulate.cc 

OBJ_SIMULATOR	= simulate.o 

OBJS_PROBLEM	= $(SRCS_PROBLEM:$(PROBLEM)/%.cc=%.o)

SRCSCODE = $(SRCS_PLANNER) $(SRCS_PROBLEM)
HDRSCODE = $(HDRS_PLANNER) $(HDRS_PROBLEM)
OBJSCODE = $(OBJS_PLANNER) $(OBJS_PROBLEM)

# ----------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------
.PHONY: all
all:		$(TARGET_SOLVER) $(TARGET_SIMULATOR)

$(TARGET_SOLVER): $(OBJSCODE) $(OBJ_SOLVER) 
		$(LINKER) $(LIBPATH) $(OBJSCODE) $(OBJ_SOLVER) $(LDFLAGS) -o $@

$(TARGET_SIMULATOR): $(OBJSCODE) $(OBJ_SIMULATOR) 
		$(LINKER) $(LIBPATH) $(OBJSCODE) $(OBJ_SIMULATOR) $(LDFLAGS) -o $@
		
$(OBJSCODE):
		echo $<
		$(CXX)  $(CXXFLAGS) -o $@ -c $<

#$(OBJS_PLANNER):
#		$(CXX)  $(CXXFLAGS) -o $@ -c $<

#$(OBJS_SIMULATOR):
#		$(CXX)  $(CXXFLAGS) -o $@ -c $<
	
clean:;	rm -f $(OBJSCODE)	
		rm -f $(OBJ_SOLVER)
		rm -f $(OBJ_SIMULATOR)
		rm -f $(TARGET_SOLVER)
		rm -f $(TARGET_SIMULATOR)

#depend:;	makedepend -Y $(INCDIR) $(SRCS)

#.PHONY:		parser
#parser:;	cd $(ParserDIR) && $(MAKE)

depend:;	g++ -MM -DNO_FREETYPE $(INCDIRS) $(HDRSCODE) $(SRCSCODE) $(SRCS_SOLVER) $(SRCS_SIMULATOR) > $(DEPFILE)

include $(DEPFILE)

# DO NOT DELETE
