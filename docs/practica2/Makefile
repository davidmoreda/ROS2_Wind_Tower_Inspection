DOC := memoria
LATEX := latexmk
LATEX_FLAGS := -pdf -interaction=nonstopmode -halt-on-error

.PHONY: all clean watch view

all: $(DOC).pdf

$(DOC).pdf: $(DOC).tex preamble.tex
	$(LATEX) $(LATEX_FLAGS) $(DOC).tex

watch:
	$(LATEX) -pvc $(LATEX_FLAGS) $(DOC).tex

view: $(DOC).pdf
	xdg-open $(DOC).pdf >/dev/null 2>&1 &

clean:
	$(LATEX) -C $(DOC).tex
	rm -f $(DOC).bbl $(DOC).run.xml
