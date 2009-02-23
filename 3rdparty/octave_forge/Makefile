all: installed

# TODO: update to download the packages individually
HTTP_MISCELLANEOUS = "http://pr.willowgarage.com/downloads/miscellaneous-1.0.7.tar.gz"
HTTP_OPTIM         = "http://pr.willowgarage.com/downloads/optim-1.0.4.tar.gz"
HTTP_SIGNAL        = "http://pr.willowgarage.com/downloads/signal-1.0.8.tar.gz"
HTTP_SPECFUN       = "http://pr.willowgarage.com/downloads/specfun-1.0.7.tar.gz"

TAR_MISCELLANEOUS = miscellaneous-1.0.7.tar.gz
TAR_OPTIM         = optim-1.0.4.tar.gz
TAR_SIGNAL        = signal-1.0.8.tar.gz
TAR_SPECFUN       = specfun-1.0.7.tar.gz

DIR_MISCELLANEOUS = miscellaneous-1.0.7
DIR_OPTIM         = optim-1.0.4
DIR_SIGNAL        = signal-1.0.8
DIR_SPECFUN       = specfun-1.0.7

HTTP_CMD  = wget
BUILD_DIR = build
TAR_CMD   = tar xvfz
#INSTALL_DIR = $(shell rospack find octave_forge)/octave-forge

installed: Makefile
	make miscellaneous
	make optim
	make signal
	make specfun
	touch installed


miscellaneous:
	mkdir -p $(BUILD_DIR)
	@if [ ! -f $(BUILD_DIR)/$(TAR_MISCELLANEOUS) ]; then cd $(BUILD_DIR) && $(HTTP_CMD) $(HTTP_MISCELLANEOUS); fi
	if [ ! -d $(BUILD_DIR)/$(DIR_MISCELLANEOUS) ]; then cd $(BUILD_DIR) && $(TAR_CMD) $(TAR_MISCELLANEOUS); fi
	export MKOCTFILE=mkoctfile && cd $(BUILD_DIR)/$(DIR_MISCELLANEOUS) && make && make install

optim:
	mkdir -p $(BUILD_DIR)
	@if [ ! -f $(BUILD_DIR)/$(TAR_OPTIM) ]; then cd $(BUILD_DIR) && $(HTTP_CMD) $(HTTP_OPTIM); fi
	if [ ! -d $(BUILD_DIR)/$(DIR_OPTIM) ]; then cd $(BUILD_DIR) && $(TAR_CMD) $(TAR_OPTIM); fi
	cd $(BUILD_DIR)/$(DIR_OPTIM) && make && make install

signal:
	mkdir -p $(BUILD_DIR)
	@if [ ! -f $(BUILD_DIR)/$(TAR_SIGNAL) ]; then cd $(BUILD_DIR) && $(HTTP_CMD) $(HTTP_SIGNAL); fi
	if [ ! -d $(BUILD_DIR)/$(DIR_SIGNAL) ]; then cd $(BUILD_DIR) && $(TAR_CMD) $(TAR_SIGNAL); fi
	cd $(BUILD_DIR)/$(DIR_SIGNAL) && make && make install

specfun:
	mkdir -p $(BUILD_DIR)
	@if [ ! -f $(BUILD_DIR)/$(TAR_SPECFUN) ]; then cd $(BUILD_DIR) && $(HTTP_CMD) $(HTTP_SPECFUN); fi
	if [ ! -d $(BUILD_DIR)/$(DIR_SPECFUN) ]; then cd $(BUILD_DIR) && $(TAR_CMD) $(TAR_SPECFUN); fi
	cd $(BUILD_DIR)/$(DIR_SPECFUN) && make && make install


clean:
	-cd $(BUILD_DIR)/$(DIR_MISCELLANEOUS) && make clean
	-cd $(BUILD_DIR)/$(DIR_OPTIM) && make clean
	-cd $(BUILD_DIR)/$(DIR_SIGNAL) && make clean
	-cd $(BUILD_DIR)/$(DIR_SPECFUN) && make clean
	rm -f installed

wipe: clean
	rm -rf $(BUILD_DIR)


