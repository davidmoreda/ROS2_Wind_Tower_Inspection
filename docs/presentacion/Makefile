DOC := presentacion
LATEX := latexmk
LATEX_FLAGS := -pdf -interaction=nonstopmode -halt-on-error

.PHONY: all clean watch view handout

all: $(DOC).pdf

$(DOC).pdf: $(DOC).tex preamble.tex
	$(LATEX) $(LATEX_FLAGS) $(DOC).tex

watch:
	$(LATEX) -pvc $(LATEX_FLAGS) $(DOC).tex

view: $(DOC).pdf
	xdg-open $(DOC).pdf >/dev/null 2>&1 &

# Version handout (sin animaciones, 4 slides por hoja A4).
handout:
	$(LATEX) $(LATEX_FLAGS) -jobname=$(DOC)-handout \
	  '\PassOptionsToClass{handout}{beamer}\input{$(DOC).tex}'

clean:
	$(LATEX) -C $(DOC).tex
	rm -f $(DOC).bbl $(DOC).run.xml $(DOC)-handout.pdf
