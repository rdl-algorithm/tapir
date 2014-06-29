# ----------------------------------------------------------------------
# Configuration of code cleaners.
# ----------------------------------------------------------------------
BEAUTIFY_EXCLUDES := ./src/problems/shared/ProgramOptions.hpp
BEAUTIFY_EXCLUDES += ./src/problems/rocksample/RockSampleOptions.hpp
BEAUTIFY_EXCLUDES += ./src/problems/tag/TagOptions.hpp
BEAUTIFY_EXCLUDES += ./src/problems/nav2d/UnderwaterNavOptions.hpp
BEAUTIFY_CFG := $(ROOT)/.make/uncrustify.cfg
BEAUTIFY_CMD := uncrustify -c $(BEAUTIFY_CFG)
BEAUTIFY_FLAGS := --no-backup

IWYU_MAPPING_FILE := $(ROOT)/.make/mappings.imp
IWYU_CMD := include-what-you-use
IWYU_FLAGS := -Xiwyu --mapping_file=$(IWYU_MAPPING_FILE) -Xiwyu --verbose=3

IWYU_FIX_CMD   := fix-includes
IWYU_FIX_FLAGS := --separate_c_cxx
IWYU_FIX_FLAGS += --nosafe_headers --comments $(INCDIRS)
IWYU_FIX_FLAGS += -o $(dir $@)

# ----------------------------------------------------------------------
# Code cleaner recipes.
# ----------------------------------------------------------------------
BEAUTIFY_RECIPE := $(BEAUTIFY_CMD) $(BEAUTIFY_FLAGS) $<
IWYU_CXX_RECIPE := $(IWYU_CMD) $(IWYU_FLAGS) $(CPPFLAGS) $(CXXFLAGS_BASE) $< 2>&1 | tee $@
IWYU_CC_RECIPE := $(IWYU_CMD) $(IWYU_FLAGS) $(CPPFLAGS) $(CFLAGS) $< 2>&1 | tee $@
IWYU_FIX_RECIPE := $(IWYU_FIX_CMD) $(IWYU_FIX_FLAGS) < $< 2>&1 | tee $@
IWYU_FORCE_RECIPE := echo "\#include \"../EMPTY_HEADER.hpp\"" >> $<
IWYU_DOFIX_RECIPE := cp -p $(dir $@)/$* $<

# ----------------------------------------------------------------------
# Code cleaning targets.
# ----------------------------------------------------------------------
.PHONY: beautify-all iwyu-all iwyu-fix-all iwyu-force-all iwyu-dofix-all iwyu-clean-all
.PHONY: beautify iwyu iwyu-fix iwyu-force iwyu-dofix iwyu-clean

beautify: beautify-all ;
iwyu: iwyu-all ;
iwyu-fix: iwyu-fix-all ;
iwyu-force: iwyu-force-all ;
iwyu-dofix: iwyu-dofix-all ;
iwyu-clean: iwyu-clean-all ;

iwyu-clean-all:
	@rm -rf iwyu-out
clean: iwyu-clean-all
