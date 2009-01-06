#
# choose between intel compiler and gcc 

# 
#CC = gcc-4.2
#CFLAGS=-O3 -ffast-math -fomit-frame-pointer -fPIC -mtune=k8 -march=k8 -Werror

#CC=/agbs/share/sw/icc/bin/icc
#CFLAGS= -fast -DCOMPILE_WITH_ICC -Werror

CC=gcc
CFLAGS=-O3 -Werror

VERBOSEFLAG=-DKMEANS_VERBOSE=1 # 0: silent, 1:iteration counter, 2:everything
PRECISION=-DINPUT_TYPE=0 # 0: double, 1:float 

MATLABDIR=/agbs/share/sw/matlab

TARGETS=mpi_assign mpi_kmeans_mex
LIBS=-lm -lmex
INCLUDES=-I$(MATLABDIR)/extern/include

# 32 bit version
SUFFIX=mexglx
LDIRS=-L$(MATLABDIR)/bin/glnx86 

# 64 bit version
#SUFFIX=mexa64
#LDIRS=-L$(MATLABDIR)/bin/glnxa64

all: $(TARGETS:%=%.$(SUFFIX)) shared

#mpi_kmeans.o: mpi_kmeans.cxx mpi_kmeans.h
#	$(CC) $(CFLAGS)  -c -o mpi_kmeans.o mpi_kmeans.cxx

#mpi_assign.o:	mpi_assign.cxx 
#	$(CC) $(CFLAGS) $(INCLUDES) $(VERBOSEFLAG) -c -o mpi_assign.o mpi_assign.cxx

%.o:	%.cxx  
	$(CC) $(CFLAGS) $(INCLUDES) $(VERBOSEFLAG) $(PRECISION) -c -o $@ $*.cxx

mpi_kmeans_mex.$(SUFFIX):	mpi_kmeans.o mpi_kmeans.h mpi_kmeans_mex.o #mpi_kmeans_mex.h
	$(CC) mpi_kmeans_mex.o $(LDIRS) $(LIBS) -shared -o mpi_kmeans_mex.$(SUFFIX) mpi_kmeans.o

mpi_assign.$(SUFFIX):	mpi_kmeans.o mpi_assign.o mpi_kmeans.h
	$(CC) mpi_assign.o $(LDIRS) $(LIBS) -shared -o mpi_assign.$(SUFFIX) mpi_kmeans.o

shared:	mpi_kmeans.cxx mpi_kmeans.h
	$(CC) -shared -Wl,-soname=libmpikmeans.so -fPIC $(CFLAGS) -o libmpikmeans.so $(VERBOSEFLAGS) $(PRECISION) mpi_kmeans.cxx
#g++ -shared -Wl,-soname=libmpikmeans.so -fPIC -O3 -Wall -o libmpikmeans.so mpi_kmeans.cxx

test:	
	matlab -nojvm -r "test_code;exit"

clean:	
	rm -f *.mexglx *.mexa64 *.o *~ *.so

