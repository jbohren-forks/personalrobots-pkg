all: trex

PLASMA:
	@if [ ! -d PLASMA ]; then svn co -r 4919 https://babelfish.arc.nasa.gov/svn/europa/PLASMA/trunk PLASMA; fi

trex: PLASMA
	@if [ ! -d TREX ]; then svn co http://prdev.willowgarage.com/svn/scratch/TREX TREX; fi
	@if [ ! -d logs ]; then mkdir logs; fi

clean:
	rm -rf PLASMA TREX trex EUROPA logs

