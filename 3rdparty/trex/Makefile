# This make file will download EUROPA and TREX from external svn repositories. It will also
# build all required libraries for TREX (and hence EUROPA) and can verify correct installation
#
# Installation:
# 1. jam must be installed. It is the build system for EUROPA and TREX. Try something like: apt-get install
# 2. source the configuration file in this directory. This sets key enviroment variables: source config
# 3. make trex
# 4. make test

# This little voodoo allows for the 'svn' command to be overriden from the
# environment, as the autobuilder does.
ifeq ($(strip $(SVN_CMDLINE)),)
SVN_CMDLINE = svn
endif

all: trex PlanWorks includes

includes: trex PlanWorks
	./copyIncludes.sh


# GET PLASMA
PLASMA:
	@if [ ! -d PLASMA ]; then $(SVN_CMDLINE) co https://babelfish.arc.nasa.gov/svn/europa/PLASMA/trunk PLASMA; fi

# GET PlanWorks
PlanWorks:
	@if [ ! -d PlanWorks ]; then $(SVN_CMDLINE) co -r 4919 https://babelfish.arc.nasa.gov/svn/europa/PlanWorks/trunk PlanWorks; fi

# Get TREX
trex: PLASMA
	@if [ ! -d TREX ]; then $(SVN_CMDLINE) co -r 7 http://trex-autonomy.googlecode.com/svn/trunk TREX; fi
	@if [ ! -d logs ]; then mkdir logs; fi
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex; fi

# Build library and run regression tests
test: trex
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else cd TREX && jam agent-inputs && jam run-agent-module-tests; fi

clean:
	rm -rf PLASMA TREX trex EUROPA logs PlanWorks include

