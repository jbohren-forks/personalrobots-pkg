all: build/Makefile
	cd build && make -s
build/Makefile:
	mkdir -p build
	cd build && cmake ..
clean:
	if [ -f build/Makefile ]; then cd build && make clean && cd ..; fi
	rm -rf build bin
