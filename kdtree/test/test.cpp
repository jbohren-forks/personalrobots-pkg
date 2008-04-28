#include "kdtree/kdtree2.hpp"

#include <boost/multi_array.hpp>
#include <boost/random.hpp>
#include <sys/time.h>

using namespace boost; 

  double timeval_diff (struct timeval x, 
		       struct timeval y)  {
    double dsec = x.tv_sec - y.tv_sec;
    double dusec = x.tv_usec - y.tv_usec;
    return dsec + dusec/1000000.0;
  }

static minstd_rand generator(42u); 
static uniform_real<> uni_dist(0,1); 
variate_generator<minstd_rand&,uniform_real<> > uni(generator,uni_dist); 

float random_variate() {
  // between [0,1)
  return(uni()); 
}

//
// define, for convenience a 2d array of floats. 
//
typedef multi_array<float,2> array2dfloat;

int main() {
  array2dfloat realdata; 

  kdtree2* tree;

  int N = 1000000;
  int dim = 3;

  struct timeval t0, t1; 

  realdata.resize(extents[N][dim]); 
    
  for (int i=0; i<N; i++) {
    for (int j=0; j<dim; j++) 
      realdata[i][j] = random_variate();
  }
  
  kdtree2_result_vector result, resultbrute;

  cout << "Building tree... ";

  gettimeofday(&t0, NULL);

  tree = new kdtree2(realdata,true);
  tree->sort_results = false;

  gettimeofday(&t1, NULL);
  cout << timeval_diff(t1, t0) << endl;

  int nn = 1;

  //  tree->n_nearest_around_point(150000,1,nn,result);

  cout << "Finding nearest... ";

  gettimeofday(&t0, NULL);
  for (int k = 0; k < N;k++) {
    tree->n_nearest_around_point(k,1,nn,result);
  }
  gettimeofday(&t1, NULL);
  cout << timeval_diff(t1, t0) << endl;

  /*
  for (int k=0; k<nn; k++) {
    cout << "Result " << k << ", " << result[k].dis << ", " << result[k].idx << endl;
  }
  */
}

