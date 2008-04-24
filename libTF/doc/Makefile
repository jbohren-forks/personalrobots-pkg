all: manual


manual: libTF_Manual.tex
	pdflatex $^
	makeindex libTF_Manual.idx
	pdflatex $^

clean:
	rm -f *~ *.aux *.toc *.log *.ilg *.ind *.idx *.out
	rm -f libTF_Manual.pdf