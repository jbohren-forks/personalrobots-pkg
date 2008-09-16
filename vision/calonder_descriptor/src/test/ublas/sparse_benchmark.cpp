#include "timer.h"
#include "rng.h"
#include "signature.h"
#include "least_distance.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <ext/algorithm> // random_sample_n is non-standard
#include <algorithm>
#include <ext/numeric> // iota is non-standard
#include <numeric>
#include <ctime>
#include <cstdio>

namespace ublas = boost::numeric::ublas;
using namespace __gnu_cxx; // for STL extensions
using namespace features;

static const int NUM_VECTORS = 500;
static const int SIG_SIZE = 600;
static const int NNZ = 50;
static const int ITERATIONS = 5;

void randomSparseVector(SparseSignature &vec,
                        features::Rng &rng, int range[SIG_SIZE])
{
  int indices[NNZ];
  random_sample_n(range, range + SIG_SIZE, indices, NNZ, rng);

  for (int i = 0; i < NNZ; ++i) {
    vec( indices[i] ) = rng.uniform();
  }
}

int main( int argc, char** argv )
{
  std::vector< SparseSignature > signatures;
  std::vector< SparseSignature > queries;
  //features::Rng rng( std::time(NULL) );
  features::Rng rng( 42 );
  int range[SIG_SIZE];

  // Fill signatures with random data
  iota(range, range + SIG_SIZE, 0);
  signatures.reserve(NUM_VECTORS);
  queries.reserve(NUM_VECTORS);
  for (int i = 0; i < NUM_VECTORS; ++i) {
    SparseSignature new_sig(SIG_SIZE, NNZ);
    randomSparseVector(new_sig, rng, range);
    signatures.push_back(new_sig);
    new_sig.clear();
    randomSparseVector(new_sig, rng, range);
    queries.push_back(new_sig);
  }

  // Timing
  float best_distance = 1000;
  {
    Timer timer("Sparse vector distance");
    for (int iter = 0; iter < ITERATIONS; ++iter) {
      BOOST_FOREACH( SparseSignature &sig, signatures ) {
        BOOST_FOREACH( SparseSignature &query, queries) {
          //float d = euclideanDistance(query, sig);
          //float d = squaredDistance(query, sig);
          //best_distance = std::min(d, best_distance);
          leastSquaredDistance(query, sig, best_distance);
        }
      }
    }
  }
  printf("Best distance: %f\n", best_distance);

  return 0;
}
