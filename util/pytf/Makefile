test:
	swig -python -c++ -v  -module tf test.i
	g++ -c test.cpp test_wrap.cxx -I/usr/include/python2.5

CFLAGS = $(shell rospack --lang=cpp --attrib=cflags export pytf)
LFLAGS = $(shell rospack --lang=cpp --attrib=lflags export pytf)

swig:
	swig -python -c++ $(CFLAGS) tf_swig.i
	g++ -c ../tf/src/tf.cpp tf_swig_wrap.cxx -I/usr/include/python2.5 `rospack --lang=cpp --attrib=cflags export tf`
	g++ -shared tf_swig_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o _tf_swig.so
