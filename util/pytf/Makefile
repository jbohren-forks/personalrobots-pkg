CFLAGS = $(shell rospack --lang=cpp --attrib=cflags export pytf)
LFLAGS = $(shell rospack --lang=cpp --attrib=lflags export pytf)

pytf:
	swig -python -c++ $(CFLAGS) pytf.i
	g++ -c pytf_wrap.cxx -I/usr/include/python2.5 `rospack --lang=cpp --attrib=cflags export tf`
	g++ -shared pytf_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o _pytf.so

