CFLAGS = $(shell rospack export --lang=cpp --attrib=cflags pytf)
LFLAGS = $(shell rospack export --lang=cpp --attrib=lflags pytf)
SWIG_FLAGS = $(shell rospack export --lang=swig --attrib=flags pytf)
pytf:
	mkdir -p src
	swig -python -c++ $(CFLAGS) pytf_swig.i
	g++ -c pytf_swig_wrap.cxx -fPIC -I/usr/include/python2.5 $(CFLAGS) 
	g++ -shared pytf_swig_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o src/_pytf_swig.so
	mv pytf_swig.py src/

clean:
	rm -rf *.so *.cxx *.o pytf_swig.py* pybullet_swig.py* src/*.so

lisptf:
	swig -cffi -c++ -module lisptf lisptf_swig.i 
#	g++ -c pytf_swig_wrap.cxx -I/usr/include/python2.5 `rospack export --lang=cpp --attrib=cflags tf`
#	g++ -shared pytf_swig_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o _pytf_swig.so
