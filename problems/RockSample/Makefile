
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
PROBLEM			= $(ROOT)/problems/underwaterNavModif/

# ----------------------------------------------------------------------
# Compiler & linker
# ----------------------------------------------------------------------
CC            = gcc 

CXX           = g++ 

LINKER	      = g++ 
		
#---------------------------------------------------------------------#
# Compiler flags
#---------------------------------------------------------------------#
 
#CXXFLAGS =  -DNO_FREETYPE `freetype-config --cflags`  
CXXFLAGS = 	-g -frounding-math -O3 $(INCDIRS)
#CXXFLAGS  =  $(INCDIRS)
 
INCDIRS	= -DDISTL1 -I$(PLANNER) -I$(PROBLEM) \
#-I/usr/local/include/opencv/ \

#---------------------------------------------------------------------#
# Linker flags
#---------------------------------------------------------------------#

LIBPATH = -L$(PROBLEM) \
#-L/usr/local/lib/ \
						
LDFLAGS = -g -w \
#-lopencv_legacy -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml \
		
#---------------------------------------------------------------------#
# Archiver
#---------------------------------------------------------------------#

ARCHIVER      = ar

ARFLAGS       = -ruv


# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

SRCS_PLANNER 	= 	$(PLANNER)/GlobalResources.cc \
					$(PLANNER)/RandGen.cc \
					$(PLANNER)/State.cc \
					$(PLANNER)/StatePool.cc \
					$(PLANNER)/HistoryEntry.cc \
					$(PLANNER)/HistorySeq.cc \
					$(PLANNER)/Histories.cc \
					$(PLANNER)/Action.cc \
					$(PLANNER)/Observation.cc \
					$(PLANNER)/BeliefNode.cc \
					$(PLANNER)/BeliefTree.cc \
					$(PLANNER)/Solver.cc 

HDRS_PLANNER 	= 	$(PLANNER)/Model.h \
					$(PLANNER)/GlobalResources.h \
					$(PLANNER)/RandGen.h \
					$(PLANNER)/State.h \
					$(PLANNER)/StatePool.h \
					$(PLANNER)/HistoryEntry.h \
					$(PLANNER)/HistorySeq.h \
					$(PLANNER)/Histories.h \
					$(PLANNER)/Action.h \
					$(PLANNER)/Observation.h \
					$(PLANNER)/BeliefNode.h \
					$(PLANNER)/BeliefTree.h \
					$(PLANNER)/Solver.h 

OBJS_PLANNER	= 	$(SRCS_PLANNER:$(PLANNER)/%.cc=%.o)

SRCS_PROBLEM	= 	$(PROBLEM)/UnderwaterNavModifModel.cc \
					$(PROBLEM)/StRoadmap.cc 

HDRS_PROBLEM	= 	$(PROBLEM)/UnderwaterNavModifModel.h \
					$(PROBLEM)/StRoadmap.h
		
OBJS_PROBLEM	= 	$(SRCS_PPROBLEM:$(PROBLEM)/%.cc=%.o)

TARGET_SOLVER  	= solve

SRCS_SOLVER		= $(PROBLEM)/solve.cc 

OBJ_SOLVER		= solve.o 

TARGET_SIMULATOR= simulate

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
