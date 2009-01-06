#include <iostream>
#include <fstream>
#include "mcutils/mcutils.h"

#define OBJ_ORIENTED 1
#if OBJ_ORIENTED
   #include "fast_kmeans.h"
   using namespace features;
#else
   #include "mpi_kmeans.h"
#endif

using namespace mcu;

int main(int argc, char* argv[])
{       
   typedef double DataT;
   DataT *X = new DataT[2000];
   DataT *CX = new DataT[2*2];
   readMat("1000x2_mat.txt", X, 2, 1000);
   unsigned int *ix = new unsigned int[1000];   
   double sse;
      
   #if OBJ_ORIENTED
      // OO way
      //
      // double run(float* features, uint size, uint dimension, uint k, int* membership, float* centroids)
      FastKMeans<DataT> kmeans;
      kmeans.setInitMethod(FastKMeans<DataT>::IM_RANDOM);
      sse = kmeans.run(X, 1000, 2, 2, ix, CX);
   #else
      // procedural way
      //
      // double kmeans(double *CXp, const double *X, unsigned int *c, unsigned int dim,
      //               unsigned int npts, unsigned int nclus, unsigned int maxiter, unsigned int nr_restarts);
      sse = kmeans(CX, X, ix, 2, 1000, 2, 100000, 10);
   #endif   

   saveMat("ix.txt", ix, 1000, 1, "%i");
   saveMat("CX.txt", CX, 2, 2, "%.10e");
   printf("sse = %.8e\n", sse);

   delete [] X;
   delete [] ix;
   
   return 0;
}

