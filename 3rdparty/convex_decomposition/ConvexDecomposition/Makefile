

OBJS = DecomposeSample.o \
	ConvexDecomposition/bestfit.o              ConvexDecomposition/float_math.o \
	ConvexDecomposition/bestfitobb.o           ConvexDecomposition/meshvolume.o \
	ConvexDecomposition/cd_hull.o              ConvexDecomposition/planetri.o \
	ConvexDecomposition/cd_wavefront.o         ConvexDecomposition/raytri.o \
	ConvexDecomposition/concavity.o            ConvexDecomposition/splitplane.o \
	ConvexDecomposition/ConvexDecomposition.o  ConvexDecomposition/triangulate.o \
	ConvexDecomposition/fitsphere.o            ConvexDecomposition/vlookup.o

HEADERS = \
	ConvexDecomposition/bestfit.h \
	ConvexDecomposition/bestfitobb.h \
	ConvexDecomposition/cd_hull.h \
	ConvexDecomposition/cd_vector.h \
	ConvexDecomposition/cd_wavefront.h \
	ConvexDecomposition/concavity.h \
	ConvexDecomposition/ConvexDecomposition.h \
	ConvexDecomposition/fitsphere.h \
	ConvexDecomposition/float_math.h \
	ConvexDecomposition/meshvolume.h \
	ConvexDecomposition/planetri.h \
	ConvexDecomposition/raytri.h \
	ConvexDecomposition/splitplane.h \
	ConvexDecomposition/triangulate.h \
	ConvexDecomposition/vlookup.h

CC = g++

DEBUG = -g

CFLAGS = -Wall -c $(DEBUG)

LFLAGS = -Wall $(DEBUG)

test: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o test

DecomposeSample.o: $(HEADERS)
	$(CC) $(CFLAGS) DecomposeSample.cpp

ConvexDecomposition/bestfit.o: ConvexDecomposition/bestfit.h
	$(CC) $(CFLAGS) ConvexDecomposition/bestfit.cpp

ConvexDecomposition/bestfitobb.o: ConvexDecomposition/bestfitobb.h
	$(CC) $(CFLAGS) ConvexDecomposition/bestfitobb.cpp

ConvexDecomposition/cd_hull.o: ConvexDecomposition/cd_hull.h
	$(CC) $(CFLAGS) ConvexDecomposition/cd_hull.cpp

ConvexDecomposition/cd_wavefront.o: ConvexDecomposition/cd_wavefront.h
	$(CC) $(CFLAGS) ConvexDecomposition/cd_wavefront.cpp

ConvexDecomposition/concavity.o: ConvexDecomposition/concavity.h
	$(CC) $(CFLAGS) ConvexDecomposition/concavity.cpp

ConvexDecomposition/ConvexDecomposition.o: ConvexDecomposition/ConvexDecomposition.h
	$(CC) $(CFLAGS) ConvexDecomposition/ConvexDecomposition.cpp

ConvexDecomposition/fitsphere.o: ConvexDecomposition/fitsphere.h
	$(CC) $(CFLAGS) ConvexDecomposition/fitsphere.cpp

ConvexDecomposition/float_math.o: ConvexDecomposition/float_math.h
	$(CC) $(CFLAGS) ConvexDecomposition/float_math.cpp

ConvexDecomposition/meshvolume.o: ConvexDecomposition/meshvolume.h
	$(CC) $(CFLAGS) ConvexDecomposition/meshvolume.cpp

ConvexDecomposition/planetri.o: ConvexDecomposition/planetri.h
	$(CC) $(CFLAGS) ConvexDecomposition/planetri.cpp

ConvexDecomposition/raytri.o: ConvexDecomposition/raytri.h
	$(CC) $(CFLAGS) ConvexDecomposition/raytri.cpp

ConvexDecomposition/splitplane.o: ConvexDecomposition/splitplane.h
	$(CC) $(CFLAGS) ConvexDecomposition/splitplane.cpp

ConvexDecomposition/triangulate.o: ConvexDecomposition/triangulate.h
	$(CC) $(CFLAGS) ConvexDecomposition/triangulate.cpp

ConvexDecomposition/vlookup.o: ConvexDecomposition/vlookup.h
	$(CC) $(CFLAGS) ConvexDecomposition/vlookup.cpp

clean:
	\rm */*.o *~ test

tar:
	tar cfv ConvexDecomposition.tar DecomposeSample.cpp test Makefile \
		ConvexDecomposition
