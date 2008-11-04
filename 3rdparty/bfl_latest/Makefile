.PHONY: $(SVN_DIR) bfl

all: bfl

SVN_DIR = bfl-svn
INSTALL_DIR = bfl-boost
CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR)/ \
	-DMATRIX_LIB=boost \
	-DRNG_LIB=boost



build: wiped bfl

.PHONY: $(SVN_DIR)
$(SVN_DIR):
	if [ ! -d $(SVN_DIR) ]; then \
	  svn co http://svn.mech.kuleuven.be:/repos/orocos/trunk/bfl bfl-svn; \
	else \
	  cd $(SVN_DIR) && svn up; \
	fi


configured: $(SVN_DIR) Makefile
	mkdir -p $(SVN_DIR)/build-boost
	cd $(SVN_DIR)/build-boost && $(CMAKE) $(CMAKE_ARGS) ..
	touch configured


bfl:	configured
	cd $(SVN_DIR)/build-boost && make $(PARALLEL_JOBS) && make install

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf configured

