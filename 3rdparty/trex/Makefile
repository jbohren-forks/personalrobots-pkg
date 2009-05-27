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

#all: trex plasma planworks includes
all: installed

installed: Makefile
	make trex planworks plasma 
	./copyIncludes.sh

includes: trex PlanWorks PLASMA
	./copyIncludes.sh


# GET plasma
PLASMA_REVISION=5378
plasma:
	@if [ ! -d PLASMA ]; then $(SVN_CMDLINE) co -r $(PLASMA_REVISION) http://europa-pso.googlecode.com/svn/PLASMA/trunk PLASMA; fi ;
	@if [ ! -n "`svn info PLASMA | grep 'Revision: $(PLASMA_REVISION)'`" ]; then cd PLASMA && $(SVN_CMDLINE) up -r $(PLASMA_REVISION); fi;

#install the patch
#patch-PLASMA: PLASMA
#	$(MAKE) PLASMA 
#	patch -f -p0 < PLASMA_PATCH.diff
#	touch patch-PLASMA 

# GET PlanWorks
PLANWORKS_REVISION=5117
planworks:
	@if [ ! -d PlanWorks ]; then $(SVN_CMDLINE) co -r $(PLANWORKS_REVISION) http://europa-pso.googlecode.com/svn/PlanWorks/trunk PlanWorks; fi ;
	@if [ ! -n "`svn info PlanWorks | grep 'Revision: $(PLANWORKS_REVISION)'`" ]; then cd PlanWorks && $(SVN_CMDLINE) up -r $(PLANWORKS_REVISION); fi;

# Get TREXA
TREX_REVISION=117
trex: plasma
	@if [ ! -d TREX ]; then $(SVN_CMDLINE) co -r $(TREX_REVISION) http://trex-autonomy.googlecode.com/svn/trunk TREX; fi ;
	@if [ ! -n "`svn info TREX | grep 'Revision: $(TREX_REVISION)'`" ]; then cd TREX && $(SVN_CMDLINE) up -r $(TREX_REVISION); fi;
	@if [ ! -d logs ]; then mkdir logs; fi
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex; fi


# Build library and run regression tests
test: trex
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex test; fi

clean:
	rm -rf PLASMA plasma TREX trex EUROPA logs PlanWorks planworks include installed

wipe: clean
