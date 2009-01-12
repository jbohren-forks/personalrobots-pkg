#include "place_recognition/kmeans.h"
#include "place_recognition/fast_kmeans.h"
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <cassert>

namespace vision {

// TODO: change FastKMeans interface so don't need to do all this copying
//namespace fast_implementation {

int kmeans(const FeatureMatrix& features,
           const std::vector<unsigned int>& input,
           std::vector<int>& membership,
           FeatureMatrix& clusters,
           int k, float threshold)
{
  // TODO: change to passing features as float**
  //std::vector<float*> pfeatures;
  //pfeatures.reserve

  FeatureMatrix input_features(input.size(), features.cols());
  for (uint i = 0; i < input.size(); ++i)
    input_features.row(i) = features.row( input[i] );
  
  features::FastKMeans<float> clusterer;
  clusterer.setInitMethod(features::FastKMeans<float>::IM_KMEANSPP);
  clusterer.run(input_features.data(),
                input_features.rows(), input_features.cols(), k,
                reinterpret_cast<uint*>(&membership[0]),
                clusters.data());
  return k; // TODO: return error instead?
}

//} // namespace fast_implementation

// Hide slow kmeans implementation
namespace naive_implementation {

// Dirt simple method - take first k features as initial cluster centers
void chooseCentersFirstK(const FeatureMatrix& features,
                         const std::vector<unsigned int>& input,
                         FeatureMatrix& clusters,
                         int k)
{
  for (int i = 0; i < k; ++i)
    clusters.row(i) = features.row( input[i] );
}

int kmeans(const FeatureMatrix& features,
           const std::vector<unsigned int>& input,
           std::vector<int>& membership,
           FeatureMatrix& clusters,
           int k, float threshold)
{
  size_t N = input.size();
  size_t dim = features.cols();

  // if fewer data objects than clusters, just return the objects themselves
  if ((int)N <= k) {
    chooseCentersFirstK(features, input, clusters, N);
    for (unsigned int i = 0; i < N; ++i) membership[i] = i;
    return N;
  }

  assert(membership.size() >= N);
  assert(clusters.rows() >= k);
  assert(clusters.cols() == (int)dim);

  // Choose initial cluster centers
  chooseCentersFirstK(features, input, clusters, k);

  std::vector<size_t> new_cluster_sizes(k);
  FeatureMatrix new_clusters(k, dim);
  std::fill(membership.begin(), membership.end(), -1);
  
  int scaled_threshold = (int)(threshold * N);
  int delta;
  do {
    // Zero out new clusters and counts
    delta = 0;
    std::fill(new_cluster_sizes.begin(), new_cluster_sizes.end(), 0);
    new_clusters.setZero();

    // Assign data objects to current clusters
    for (unsigned int i = 0; i < N; ++i) {
      float d_min = std::numeric_limits<float>::max();
      int nearest = -1;
      // Find the nearest cluster center (L2 distance) to object i
      for (int j = 0; j < k; ++j) {
        float distance = distanceL2( features.row( input[i] ), clusters.row(j) );
        if (distance < d_min) {
          d_min = distance;
          nearest = j;
        }
      }

      if (membership[i] != nearest) {
        ++delta;
        membership[i] = nearest;
      }

      new_clusters.row(nearest) += features.row( input[i] );
      ++new_cluster_sizes[nearest];
    }

    // Assign new clusters
    for (int i = 0; i < k; ++i) {
      if (new_cluster_sizes[i] > 0) {
        clusters.row(i) = new_clusters.row(i) / new_cluster_sizes[i];
      }
      else {
        // Choose a new cluster randomly from the input features
        unsigned int index = rand() % N;
        clusters.row(i) = features.row( input[index] );
      }
    }
    
  } while (delta > scaled_threshold);

  return k;
}

} // namespace naive_implementation

} // namespace vision
