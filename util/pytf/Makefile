test:
	swig -python -c++ -v  -module tf test.i
	g++ -c test.cpp test_wrap.cxx -I/usr/include/python2.5

FLAGS = $(shell rospack --lang=cpp --attrib=cflags export tf)

swig:
	swig -python -c++ $(FLAGS) tf_swig.i
	g++ -c ../tf/src/tf.cpp tf_swig_wrap.cxx -I/usr/include/python2.5 `rospack --lang=cpp --attrib=cflags export tf`

