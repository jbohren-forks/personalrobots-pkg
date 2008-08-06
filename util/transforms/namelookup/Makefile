CFLAGS = `rospack export/cpp/cflags namelookup`
LFLAGS = `rospack export/cpp/lflags namelookup`

.PHONY: test

all:
	`rospack find rostools`/scripts/gensrv srv/*.srv
clean: 
	-rm -rf srv/cpp src/namelookup/srv

testClient: testClient.cc
	g++ $(CFLAGS) $^ -o testClient $(LFLAGS)

test:
	rostest test/test-namelookup-server.xml