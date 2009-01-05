#include "place_recognition/kmeans.h"
#include <Eigen/Array>
#include <boost/timer.hpp>
#include <cstdio>

using namespace vision;

int main(int argc, char** argv)
{
  static const int K = 8;
  static const int D = 4;
  static const int SAMPLES = 1000000;
  static const int N = K * SAMPLES;
  
  // Randomly select 5 centers
  FeatureMatrix centers = FeatureMatrix::Random(K, D) * 100;
  std::cout << "Centers\n" << centers << std::endl;

  // Generate noisy data
  FeatureMatrix data = FeatureMatrix::Random(N, D) * 5;
  for (int i = 0; i < SAMPLES; ++i) {
    data.block<K, D>(i * K, 0) += centers;
  }
  //std::cout << "Data\n" << data << std::endl;

  // Bookkeeping
  std::vector<unsigned int> input(N);
  for (int i = 0; i < N; ++i) input[i] = i;
  std::vector<int> membership(N);
  FeatureMatrix clusters(K, D);
  
  // Do kmeans clustering
  boost::timer t;
  int num_clusters = kmeans(data, input, membership, clusters, K);
  printf("\nkmeans: %fs\n", t.elapsed());

  std::cout << std::endl << num_clusters << " clusters:\n" << clusters << std::endl;
  
  return 0;
}
