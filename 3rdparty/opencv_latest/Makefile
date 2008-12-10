ifeq ($(strip $(CVS_CMDLINE)),)
CVS_CMDLINE = cvs
endif


CVS_DIR = opencv-cvs
INSTALL_DIR = opencv
CVS_DATE = "10/12/2008 00:00"

all: $(INSTALL_DIR)


.PHONY: $(CVS_DIR)
$(CVS_DIR):
	-$(CVS_CMDLINE) -d:pserver:anonymous:@opencvlibrary.cvs.sourceforge.net:/cvsroot/opencvlibrary login; \
	if [ ! -d $(CVS_DIR) ]; then \
	  $(CVS_CMDLINE) -z3 -d:pserver:anonymous@opencvlibrary.cvs.sourceforge.net:/cvsroot/opencvlibrary co -D $(CVS_DATE) -P -d $(CVS_DIR) opencv; \
	else \
	  cd $(CVS_DIR) && $(CVS_CMDLINE) up -D $(CVS_DATE); \
	fi

configured: $(CVS_DIR) Makefile
	@if [ ! -d /usr/include/gtk-2.0 ]; then echo "you don't appear to have the GTK+2.0 headers. On Ubuntu, this is fixed with 'sudo apt-get install libgtk2.0-dev'"; false; fi
	cd $(CVS_DIR) &&./configure --prefix=$(PWD)/$(INSTALL_DIR) --without-python
	touch configured

.PHONY: $(INSTALL_DIR)
$(INSTALL_DIR): configured
	cd $(CVS_DIR) && ln -fs /usr/bin/libtool . && make $(PARALLEL_JOBS) && make install
	@echo "patch opencv.pc to pass -Wl,-rpath,-L{libdir}"
	-mv $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak
	sed 's%Libs: -L$${libdir} -lcxcore%Libs: -Wl,-rpath,$${libdir} -L$${libdir} -lcxcore%g' $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak > $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc

clean:
	-rm -rf $(INSTALL_DIR) configured

wipe: clean
	rm -rf $(CVS_DIR)

.PHONY : clean download
