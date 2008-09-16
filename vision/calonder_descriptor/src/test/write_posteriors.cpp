#include "calonder_descriptor/rtree_classifier.h"
#include <fstream>
#include <cmath>
#include <boost/foreach.hpp>

using namespace features;

int main( int argc, char** argv )
{
  static const int BINS = 10;
  static const int LAST_BIN = BINS-1;
  long bin_counts[BINS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  
  RTreeClassifier classifier;
  classifier.read(argv[1]);
  
  BOOST_FOREACH( RandomizedTree &tree, classifier.trees_ ) {
    BOOST_FOREACH( float p, tree.posteriors_ ) {
      if (p == 0) {
        bin_counts[LAST_BIN]++;
      } else {
        int bin = (int)(-log10(p));
        if (bin > LAST_BIN)
          bin = LAST_BIN;
        bin_counts[bin]++;
      }
    }
  }

  std::ofstream file(argv[2]);
  for (int i = 0; i < BINS; ++i) {
    file << i << ' ' << bin_counts[i] << std::endl;
  }
  file.close();
  
  return 0;
}
