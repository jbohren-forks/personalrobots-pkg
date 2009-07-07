#include <descriptors_3d/local_geometry.h>

using namespace std;

inline bool LocalGeometry::readyToCompute()
{
  return data_set_ && ((interest_pt_set_ && radius_ > 0.0) || interest_region_set_);
}

bool LocalGeometry::compute(Eigen::MatrixXf** result, bool debug)
{
  if (readyToCompute() == false)
  {
    ROS_ERROR("not ready to compute");
    return false;
  }

  // ----------------------------------------
  // find neighbors
  const vector<int>* neighbor_indices = NULL;

  vector<int> range_search_indices;
  if (interest_pt_set_)
  {
    neighbor_indices = &range_search_indices;
    vector<float> neighbor_distances; // unused
    if (data_kdtree_->radiusSearch(interest_pt_idx_, radius_, range_search_indices, neighbor_distances)
        == false)
    {
      ROS_ERROR("radius search failed");
      return false;
    }
  }
  else
  {
    neighbor_indices = interest_region_indices_;
  }

  // ----------------------------------------
  if (neighbor_indices->size() < 3)
  {
    ROS_ERROR("did not have enough neighbors");
    return false;
  }

  // ----------------------------------------
  Eigen::Matrix3d eigen_vectors;
  Eigen::Vector3d eigen_values;
  Eigen::Vector3d tangent;
  Eigen::Vector3d normal;
  cloud_geometry::nearest::computePatchEigen(*data_, *neighbor_indices, eigen_vectors, eigen_values);
  // smallest eigenvalue = index 0
  for (int i = 0 ; i < 3 ; i++)
  {
    normal[i] = eigen_vectors(i, 0);
    tangent[i] = eigen_vectors(i, 2);
  }

  // ----------------------------------------
  *result = new Eigen::MatrixXf(result_size_, 1);

  unsigned int idx = 0;
  (**result)[idx++] = eigen_values[2];
  (**result)[idx++] = eigen_values[2] - eigen_values[1];
  (**result)[idx++] = eigen_values[1] - eigen_values[0];

  if (ref_tangent_defined_)
  {
    double tangent_dot = tangent.dot(ref_tangent_);
    if (tangent_dot < 0.0)
    {
      tangent_dot = tangent.dot(ref_tangent_flipped_);
    }
    (**result)[idx++] = tangent_dot;
  }

  if (ref_normal_defined_)
  {
    double normal_dot = normal.dot(ref_normal_);
    if (normal_dot < 0.0)
    {
      normal_dot = normal.dot(ref_normal_flipped_);
    }
    (**result)[idx++] = normal_dot;
  }

  if (use_elevation_)
  {
    (**result)[idx++] = data_->pts[interest_pt_idx_].z;
  }

  return true;
}
