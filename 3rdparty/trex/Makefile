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

includes: trex PlanWorks PLASMA
	./copyIncludes.sh


# GET PLASMA
PLASMA:
	@if [ ! -d PLASMA ]; then $(SVN_CMDLINE) co -r 5141 http://europa-pso.googlecode.com/svn/PLASMA/trunk PLASMA; fi ;
	-@cd PLASMA && $(SVN_CMDLINE) up -r 5141 ;

#install the patch
#patch-PLASMA: PLASMA
#	$(MAKE) PLASMA 
#	patch -f -p0 < PLASMA_PATCH.diff
#	touch patch-PLASMA 

# GET PlanWorks
PlanWorks:
	@if [ ! -d PlanWorks ]; then $(SVN_CMDLINE) co -r 5117 http://europa-pso.googlecode.com/svn/PlanWorks/trunk PlanWorks; fi ;
	-@cd PlanWorks && $(SVN_CMDLINE) up -r 5117 ;

# Get TREX
trex: PLASMA
	@if [ ! -d TREX ]; then $(SVN_CMDLINE) co -r 35 http://trex-autonomy.googlecode.com/svn/trunk TREX; fi ;
	@-cd TREX && $(SVN_CMDLINE) up -r 35 ;
	@if [ ! -d logs ]; then mkdir logs; fi
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex; fi



# Build library and run regression tests
test: trex
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else cd TREX && jam agent-inputs && jam run-agent-module-tests; fi

clean:
	rm -rf PLASMA TREX trex EUROPA logs PlanWorks include

