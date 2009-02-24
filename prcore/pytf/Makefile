CFLAGS = $(shell rospack export --lang=cpp --attrib=cflags pytf)
LFLAGS = $(shell rospack export --lang=cpp --attrib=lflags pytf)

pytf:
	swig -python -c++ $(CFLAGS) pytf.i
	g++ -c pytf_wrap.cxx -I/usr/include/python2.5 `rospack export --lang=cpp --attrib=cflags tf`
	g++ -shared pytf_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o _pytf.so

clean:
	rm -rf *.so *.cxx *.o *.py*