# This make file will download EUROPA and TREX from external svn repositories. It will also
# build all required libraries for TREX (and hence EUROPA) and can verify correct installation
#
# Installation:
# 1. jam must be installed. It is the build system for EUROPA and TREX. Try something like: apt-get install
# 2. source the configuration file in this directory. This sets key enviroment variables: source config
# 3. make trex
# 4. make test

all: trex

# GET EUROPA
PLASMA:
	@if [ ! -d PLASMA ]; then svn co -r 4919 https://babelfish.arc.nasa.gov/svn/europa/PLASMA/trunk PLASMA; fi

# Get TREX
trex: PLASMA
	@if [ ! -d TREX ]; then svn co http://prdev.willowgarage.com/svn/scratch/TREX TREX; fi
	@if [ ! -d logs ]; then mkdir logs; fi

# Build library and run regression tests
test: trex
	cd TREX
	jam agent-inputs
	jam run-agent-module-tests

clean:
	rm -rf PLASMA TREX trex EUROPA logs

