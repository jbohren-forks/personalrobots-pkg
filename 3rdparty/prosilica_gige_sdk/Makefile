all: installed

TARBALL = Prosilica\ GigE\ SDK\ Linux\ 1.18.tgz
TARBALL_URL = http://www.prosilica.com/support/gige/081015/Prosilica%20GigE%20SDK%20Linux%201.18.tgz
SOURCE_DIR = Prosilica_GigE_SDK
INITIAL_DIR = Prosilica\ GigE\ SDK
# Spaces in tarball file name screw this up, awesome
#MD5SUM_FILE = $(TARBALL).md5sum
include $(shell rospack find mk)/download_unpack.mk

installed: $(SOURCE_DIR)

clean:

wipe: clean
	rm -rf $(SOURCE_DIR) $(TARBALL)
