CFLAGS = $(shell rospack export --lang=cpp --attrib=cflags bullet)
LFLAGS = $(shell rospack export --lang=cpp --attrib=lflags bullet)


pybullet:
	mkdir -p src
	cd swig && swig -python -c++ $(CFLAGS) bullet_swig.i 
	cp swig/bullet.py src/
	cd swig && g++ -c bullet_swig_wrap.cxx -I/usr/include/python2.5 `rospack export --lang=cpp --attrib=cflags tf`
	cd swig && g++ -shared bullet_swig_wrap.o $(LFLAGS) -Xlinker -export-dynamic -o ../src/_bullet.so


clean:
	rm -rf swig/*.so swig/*.cxx swig/*.o  swig/bullet.py* src/bullet.py* src/*.so
