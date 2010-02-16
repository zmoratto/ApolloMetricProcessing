all: apollo_processing.pdf

apollo_processing.pdf:
	pdflatex apollo_processing.tex
	pdflatex apollo_processing.tex

clean:
	rm -rf *.o *~ \#* *.aux *.log

distclean: clean
	rm -rf generate apollo_processing.toc apollo_processing.pdf .libs apollo_processing.out

.PHONY: images clean distclean all