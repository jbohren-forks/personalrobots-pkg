PWD = $(shell pwd)
INSTALL_DIR = $(PWD)/../lib
INCLUDE_DIR = $(PWD)/../include
CC = g++
LIBTOOL = libtool
INSTALL = install

CFLAGS = -c -g -I$(INCLUDE_DIR)
LDFLAGS = -rpath $(INSTALL_DIR)
# Optimization settings
CFLAGS += -O3 -DNDEBUG
# OpenCV
CFLAGS += `pkg-config opencv --cflags`
LDFLAGS += `pkg-config opencv --libs`
# CBLAS
LDFLAGS += -L/usr/lib -lcblas
# Boost (headers only)
CFLAGS += -I$(BOOST_ROOT)/include
# GSL
#GSL_ROOT = /usr
#CFLAGS += -I$(GSL_ROOT)/include -DHAVE_GSL
#LDFLAGS += -L$(GSL_ROOT)/lib -lgsl -lgslcblas

SOURCES = patch_generator.cpp randomized_tree.cpp rtree_classifier.cpp
OBJECTS = $(SOURCES:.cpp=.lo)
LIBNAME = libsignatures

all: $(LIBNAME).so

$(LIBNAME).so: $(OBJECTS)
	$(LIBTOOL) --mode=link $(CC) $(LDFLAGS) $(OBJECTS) -o $(LIBNAME).la
	$(LIBTOOL) --mode=install $(INSTALL) $(LIBNAME).la $(INSTALL_DIR)

$(OBJECTS): %.lo: %.cpp
	$(LIBTOOL) --mode=compile $(CC) $(CFLAGS) $<

clean:
	rm -rf $(OBJECTS) $(SOURCES:.cpp=.o) .libs $(INSTALL_DIR)/* $(LIBNAME).la

