.PHONY=all clean test


all: test



test: main.cc libTF.o
	g++ -g -O2 $^ -o test -lnewmat


clean:
	rm -rf *~ test *.o