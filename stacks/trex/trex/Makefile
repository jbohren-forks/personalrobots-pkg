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

#all: trex plasma includes
all: installed

installed: Makefile
	make trex plasma includes

includes: trex plasma 
	@if [ ! -d include ]; then echo "Try to make" ; fi

# GET plasma
PLASMA_REVISION=5634
plasma:
	@if [ -n "`svn info PLASMA | grep 'http://europa-pso.googlecode.com/svn/PLASMA/branches/willow-trex'`" ]; then echo "Your PLASMA is not of the correct branch. Reclocating and updating" ; cd PLASMA && svn switch http://europa-pso.googlecode.com/svn/PLASMA/trunk && $(SVN_CMDLINE) up -r $(PLASMA_REVISION) ; cd ../ && ./copyIncludes.sh ; cd `rospack find trex` && ./touch_nddl3.sh ; fi ;
	@if [ -n "`svn info PLASMA | grep 'https://europa-pso.googlecode.com/svn/PLASMA/branches/willow-trex'`" ]; then echo "Your PLASMA is not of the correct branch. Reclocating and updating" ; cd PLASMA && svn switch https://europa-pso.googlecode.com/svn/PLASMA/trunk && $(SVN_CMDLINE) up -r $(PLASMA_REVISION) ; cd ../ && ./copyIncludes.sh ; cd `rospack find trex` && ./touch_nddl3.sh ; fi ;
	@if [ ! -d PLASMA ]; then $(SVN_CMDLINE) co -r $(PLASMA_REVISION) http://europa-pso.googlecode.com/svn/PLASMA/trunk PLASMA; ./copyIncludes.sh ; cd `rospack find trex` && ./touch_nddl3.sh ; fi ;
	@if [ ! -n "`svn info PLASMA | grep 'Revision: $(PLASMA_REVISION)'`" ]; then cd PLASMA && $(SVN_CMDLINE) up -r $(PLASMA_REVISION) ; cd ../ && ./copyIncludes.sh ; cd `rospack find trex` && ./touch_nddl3.sh ; fi;

#install the patch
#patch-PLASMA: PLASMA
#	$(MAKE) PLASMA 
#	patch -f -p0 < PLASMA_PATCH.diff
#	touch patch-PLASMA 

# Get TREX
TREX_REVISION=231
trex: plasma
# If ROS_TREX_GOOGLE_CODE_USERNAME is defined in the environment, check
# out with that username. This will allow commits to be made directly back
# to the google code svn repository without having to copy the files to
# another checkout path.
	@if [ -n "${ROS_TREX_GOOGLE_CODE_USERNAME}" -a ! -d TREX ]; then $(SVN_CMDLINE) co -r $(TREX_REVISION) https://trex-autonomy.googlecode.com/svn/trunk TREX --username ${ROS_TREX_GOOGLE_CODE_USERNAME}; ./copyIncludes.sh ; fi ;
# If ROS_TREX_GOOGLE_CODE_USERNAME is defined in the environment, and
# If the currently checked out revision is older than the current revision,
# update it, otherwise, do not backdate the checkout.
	@if [ -n "${ROS_TREX_GOOGLE_CODE_USERNAME}" -a "`svn info TREX | grep Revision | sed 's/.\+\s\([0-9]\+\)/\1/'`" -lt $(TREX_REVISION) ]; then cd TREX && $(SVN_CMDLINE) up -r $(TREX_REVISION) --username ${ROS_TREX_GOOGLE_CODE_USERNAME}; cd ../ && ./copyIncludes.sh ; fi ;
# If ROS_TREX_GOOGLE_CODE_USERNAME is not defined, then TREX will be checked
# out anonymously if the directory does not exist.
	@if [ ! -d TREX ]; then $(SVN_CMDLINE) co -r $(TREX_REVISION) http://trex-autonomy.googlecode.com/svn/trunk TREX; ./copyIncludes.sh ; fi ;
# If ROS_TREX_GOOGLE_CODE_USERNAME is not defined, then always update the
# checkout to the target revision during a make.
	@if [ ! -n "${ROS_TREX_GOOGLE_CODE_USERNAME}" -a ! -n "`svn info TREX | grep 'Revision: $(TREX_REVISION)'`" ]; then cd TREX && $(SVN_CMDLINE) up -r $(TREX_REVISION); cd ../ && ./copyIncludes.sh ; fi;
	@if [ ! -d logs ]; then mkdir logs; fi
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex; fi

# Build library and run regression tests
test: trex
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex test; fi

clean:
	rm -rf PLASMA plasma TREX trex EUROPA logs include installed

wipe: clean
