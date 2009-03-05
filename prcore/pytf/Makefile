CFLAGS = $(shell rospack export --lang=cpp --attrib=cflags pytf)
LFLAGS = $(shell rospack export --lang=cpp --attrib=lflags pytf)

pytf:
	swig -python -c++ $(CFLAGS) pytf_swig.i 
	g++ -c pytf_swig_wrap.cxx -I/usr/include/python2.5 `rospack export --lang=cpp --attrib=cflags tf`
	g++ -shared pytf_swig_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o _pytf_swig.so

clean:
	rm -rf *.so *.cxx *.o *.py*