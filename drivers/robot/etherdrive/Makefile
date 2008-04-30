
CFLAGS += -Iinclude


all: test_etherdrive2


etherdrive.o: src/libetherdrive/etherdrive.cpp
	g++ $(CFLAGS) -c $^ -o $@

lib/libetherdrive.a: etherdrive.o
	ar -rs $@ $^ 

test_etherdrive2: test/test_etherdrive2/test_etherdrive2.cpp lib/libetherdrive.a
	g++ $(CFLAGS) $^ -o $@ 