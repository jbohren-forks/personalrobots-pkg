.PHONY=all clean test


all: test

#%.o:%.cc

#%.o:%.hh

quat: testQuat.cc Quaternion3D.o
	g++ -g -O2 $^ -o quat -lnewmat

test: main.cc libTF.o Quaternion3D.o
	g++ -g -O2 $^ -o test -lnewmat


clean:
	rm -rf *~ test *.o