MODULE_NAME = solver
CHILD_FOLDERS := indexing mappings serialization abstract-problem changes
CHILD_FOLDERS += search belief-estimators

ifdef HAS_ROOT_MAKEFILE

include .make/template.mk

# Elements that make up the library - deferred expansion!
MEMBERS_solver            = $(ALL_OBJS_solver) $(OBJS_global)

# The output file for the library.
ifeq ($(CFG),shared)
	LIB_solver               := $(BUILDDIR)/libabt.so
else
	LIB_solver               := $(BUILDDIR)/libabt.a
endif
OUTPUTS_TO_CLEAN_solver  += $(LIB_solver)

# Configuration for building the library
build-solver: $(LIB_solver)

# Now we define the command to build this library
ifeq ($(CFG),shared)
$(LIB_solver): $$(MEMBERS_solver)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -shared $(MEMBERS_solver) -o $@
else
$(LIB_solver): $$(MEMBERS_solver)
	$(AR) rcs $@ $(MEMBERS_solver)
endif

# Rebuild the archive if this Makefile changes.
$(LIB_solver): $(d)/Makefile

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS) call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
