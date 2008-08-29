# STAIR VISION LIBRARY APPLICATIONS MAKEFILE
# Stephen Gould <sgould@stanford.edu>

.PHONY: clean depend matlab ml pgm test vision

all: depend ml pgm test vision matlab

depend:
	cd gui && make depend && cd ..
	cd matlab && make depend && cd ..
	cd ml && make depend && cd ..
	cd pgm && make depend && cd ..
	cd test && make depend && cd ..
	cd vision && make depend && cd ..

gui:
	cd gui && make && cd ..

matlab:
	cd matlab && make && cd ..

ml:
	cd ml && make && cd ..

pgm:
	cd pgm && make && cd ..

test:
	cd test && make && cd ..

vision:
	cd vision && make && cd ..

clean:
	cd gui && make clean && cd ..
	cd matlab && make clean && cd ..
	cd ml && make clean && cd ..
	cd pgm && make clean && cd ..
	cd test && make clean && cd ..
	cd vision && make clean && cd ..
	find -name "*~" -delete

