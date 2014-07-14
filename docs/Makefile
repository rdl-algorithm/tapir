MODULE_NAME = docs

ifdef HAS_ROOT_MAKEFILE

# ----------------------------------------------------------------------
# Documentation targets
# ----------------------------------------------------------------------
DOC_OUTPUT_DIRS := $(ROOT)/docs/html $(ROOT)/docs/generated

define HTML_RECIPE
	@echo '<!doctype html>' > $@
	@echo '<html>' >> $@
	@echo '<head>' >> $@
	@echo '<meta charset="utf-8">' >> $@
	@echo '<style>' >> $@
	@cat $(2) >> $@
	@echo '</style>' >> $@
	@echo '<title>$(1)</title>' >> $@
	@echo '</head>' >> $@
	@echo '<body>' >> $@
	@markdown $< >> $@
	@echo '</body>' >> $@
	@echo '</html>' >> $@
	@perl -pi -e 's/\.md">/.html">/g' $@
endef

define DOXYGEN_MD_RECIPE
	@echo "# $(1) {#$(2)-top}\n" > $@
	@cat $< $(3) >> $@
	@perl -0pi -e 's/^(.*)(\n=+)$$/\1 {#$(2)}\2/mg' $@
	@perl -0pi -e 's/^(-+\n)(.*)(\n-+)$$/\1\2 {#\L\2}\3/mg' $@
	@perl -0pi -e 's/^(#{2,}\s+)(.*)$$/\1\2 {#\L\2}/mg' $@
	@perl -pi -e 's/({#.*})/($$res = $$1) =~ s\/ \/-\/g, $$res =~ s\/[()\.]\/\/g, $$res/eg' $@
endef

# Create HTML versions of the Markdown-formatted READMEs
.make/README.html: .make/README.md docs/style.css docs/Makefile
	$(call HTML_RECIPE,TAPIR Build System,docs/style.css)
README.html: README.md docs/style.css docs/Makefile
	$(call HTML_RECIPE,TAPIR Quick Start Guide,docs/style.css)
docs/README.html: docs/README.md docs/style.css docs/Makefile
	$(call HTML_RECIPE,TAPIR Overview,docs/style.css)
docs/README-MakeNewModel.html: docs/README-MakeNewModel.md docs/style.css docs/Makefile
	$(call HTML_RECIPE,Making a new model for TAPIR,docs/style.css)
GENERATED_HTML := README.html .make/README.html docs/README.html docs/README-MakeNewModel.html

# Adapt the README.md files so that they will link nicely with Doxygen.
docs/generated/Quick_Start.md: README.md docs/doxygen_links/Quick_Start.md docs/Makefile
	$(call DOXYGEN_MD_RECIPE,Quick Start Guide,quick-start,docs/doxygen_links/Quick_Start.md)
docs/generated/Overview.md: docs/README.md docs/doxygen_links/Overview.md docs/Makefile
	$(call DOXYGEN_MD_RECIPE,Overview,overview,docs/doxygen_links/Overview.md)
docs/generated/New_Model.md: docs/README-MakeNewModel.md docs/doxygen_links/New_Model.md docs/Makefile
	$(call DOXYGEN_MD_RECIPE,Making a new model,new-model,docs/doxygen_links/New_Model.md)
docs/generated/Build_System.md: .make/README.md docs/doxygen_links/Build_System.md docs/Makefile
	$(call DOXYGEN_MD_RECIPE,Build System,build-system,docs/doxygen_links/Build_System.md)

DOXYGEN_MD_FILES := docs/generated/Quick_Start.md docs/generated/Overview.md
DOXYGEN_MD_FILES += docs/generated/New_Model.md docs/generated/Build_System.md

$(DOXYGEN_MD_FILES): docs/generated
docs/generated:
	$(MKDIR_RECIPE)

# Now a command to generate the documentation.
.PHONY: docs
docs: $(GENERATED_HTML)
docs: docs/Doxyfile $(DOXYGEN_MD_FILES)
	@echo "Using Doxygen to generate documentation..."
	@doxygen docs/Doxyfile 2>&1 | grep -v "LINK_DISABLED" 1>&2 ; true
	@echo "Done."

.PHONY: clean-docs
clean-docs:
	@echo Removing documentation directories!
	@rm -rfv $(DOC_OUTPUT_DIRS) | grep "directory" ; true



else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS): call-upwards ;
call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
