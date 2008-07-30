all: manual

manual: libTF_Manual.tex
	pdflatex $^
	bibtex libTF_Manual
	makeindex libTF_Manual.idx
	pdflatex $^
	pdflatex $^

clean:
	rm -f *~ *.aux *.toc *.log *.ilg *.ind *.idx *.out *.bbl *.blg
	rm -f libTF_Manual.pdf