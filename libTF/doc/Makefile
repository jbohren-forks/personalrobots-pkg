all: manual


manual: libTF_Manual.tex
	pdflatex $^
	pdflatex $^

clean:
	rm -f *~ *.aux *.toc *.log 
	rm -f libTF_Manual.pdf