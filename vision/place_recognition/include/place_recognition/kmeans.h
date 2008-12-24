#ifndef _KMEANS_H_
#define _KMEANS_H_

#include <vector>
#include <Eigen/Core>

namespace vision {

// Want matrix in row-major order
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureMatrix;

template < typename Vector1T, typename Vector2T >
inline float distanceL2(const Eigen::MatrixBase< Vector1T >& v1,
                        const Eigen::MatrixBase< Vector2T >& v2)
{
  return (v1 - v2).squaredNorm();
}

// TODO: Generalize matrix types so Eigen can do vectorization?
int kmeans(const FeatureMatrix& features,
           const std::vector<unsigned int>& input,
           std::vector<int>& membership,
           FeatureMatrix& clusters,
           int k, float threshold = 0.0f);

} // namespace vision

#endif
