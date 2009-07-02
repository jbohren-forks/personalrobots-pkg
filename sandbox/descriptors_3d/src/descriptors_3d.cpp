#include <descriptors_3d/descriptors_3d.h>

using namespace std;

inline bool LocalGeometry::readyToCompute()
{
  return data_defined_ && interest_pt_set_ && (radius_ > 0.0 || indices_ != NULL);
}

bool LocalGeometry::compute(Eigen::MatrixXd** result, bool debug)
{
  if (readyToCompute() == false)
  {
    return false;
  }

  // find neighbors
  vector<int> neighbor_indices;
  if (radius_ > 0.0)
  {
    vector<float> neighbor_distances; // unused

    if (data_kdtree_->radiusSearch(interest_pt_idx_, radius_, neighbor_indices, neighbor_distances) == false)
    {
      return false;
    }

    indices_ = &neighbor_indices;
  }

  if (neighbor_indices.size() < 3)
  {
    return false;
  }

  Eigen::Matrix3d eigen_vectors;
  Eigen::Vector3d eigen_values;
  cloud_geometry::nearest::computePatchEigen(*data_, neighbor_indices, eigen_vectors, eigen_values);

  Eigen::Vector3d tangent;
  Eigen::Vector3d normal;

  // smallest eigenvalue = index 0
  for (int i = 0 ; i < 3 ; i++)
  {
    normal[i] = eigen_vectors(i, 0);
    tangent[i] = eigen_vectors(i, 2);
  }

  result_size_ = 3;

  double tangent_dot = -1.0;
  if (ref_tangent_defined_)
  {
    result_size_++;
    tangent_dot = tangent.dot(ref_tangent_);
    if (tangent_dot < 0.0)
    {
      tangent_dot = tangent.dot(ref_tangent_flipped_);
    }
  }

  double normal_dot = -1.0;
  if (ref_normal_defined_)
  {
    result_size_++;
    normal_dot = normal.dot(ref_normal_);
    if (tangent_dot < 0.0)
    {
      normal_dot = normal.dot(ref_normal_flipped_);
    }
  }

  if (use_elevation_)
  {
    result_size_++;
  }

  *result = new Eigen::MatrixXd(result_size_, 1);

  unsigned int idx = 0;
  (**result)[idx++] = eigen_values[2];
  (**result)[idx++] = eigen_values[2] - eigen_values[1];
  (**result)[idx++] = eigen_values[1] - eigen_values[0];

  if (ref_tangent_defined_)
  {
    (**result)[idx++] = tangent_dot;
  }

  if (ref_normal_defined_)
  {
    (**result)[idx++] = normal_dot;
  }

  if (use_elevation_)
  {
    (**result)[idx++] = data_->pts[interest_pt_idx_].z;
  }

  // reset to NULL if did range search
  if (radius_ > 0.0)
  {
    indices_ = NULL;
  }
  return true;
}
