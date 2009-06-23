#include "timer.h"
#include "rng.h"
#include "signature.h"
#include "least_distance.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
#include <ctime>
#include <cstdio>

namespace ublas = boost::numeric::ublas;
using namespace features;

static const int NUM_VECTORS = 500;
static const int SIG_SIZE = 600;
static const int ITERATIONS = 5;

void randomDenseVector(DenseSignature &vec, features::Rng &rng)
{
  for (unsigned i = 0; i < vec.size(); ++i) {
    vec(i) = rng.uniform();
  }
}

int main( int argc, char** argv )
{
  std::vector< DenseSignature > signatures;
  std::vector< DenseSignature > queries;
  //features::Rng rng( std::time(NULL) );
  features::Rng rng( 42 );

  // Fill signatures with random data
  signatures.reserve(NUM_VECTORS);
  queries.reserve(NUM_VECTORS);
  DenseSignature new_sig(SIG_SIZE);
  for (int i = 0; i < NUM_VECTORS; ++i) {
    randomDenseVector(new_sig, rng);
    signatures.push_back(new_sig);
    randomDenseVector(new_sig, rng);
    queries.push_back(new_sig);
  }

  // Timing
  float best_distance = 1000;
  {
    Timer timer("Dense vector distance");
    for (int iter = 0; iter < ITERATIONS; ++iter) {
      BOOST_FOREACH( DenseSignature &sig, signatures ) {
        BOOST_FOREACH( DenseSignature &query, queries) {
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
