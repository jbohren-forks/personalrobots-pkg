test:
	swig -python -c++ test.i
	g++ -c test.cpp test_wrap.c -I/usr/include/python2.5

swig:
	swig -python -c++ `rospack --lang=cpp --attrib=cflags export tf` tf_swig.i
	g++ -c ../tf/src/tf.cpp tf_swig_wrap.c -I/usr/include/python2.5 `rospack --lang=cpp --attrib=cflags export tf`

