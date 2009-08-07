#ifndef NEW_POINT_CLOUD_POINT_CLOUD_STRUCT_H
#define NEW_POINT_CLOUD_POINT_CLOUD_STRUCT_H

#include <Eigen/Core>
#include <boost/shared_array.hpp>
#include <boost/static_assert.hpp>
#include <opencv/cv.h>

namespace new_point_cloud {

struct Color
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
};

struct Point
{
  float x;
  float y;
  float z;
  
  union {
    Color color;
    float w;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN_128;

namespace verify_size {
  BOOST_STATIC_ASSERT(sizeof(Point) == 16);
}

struct PointCloud
{
  uint32_t width;
  uint32_t height;
  boost::shared_array<Point> points;

  PointCloud()
    : width(0), height(0), points(NULL)
  {}

  void allocate(uint32_t width_, uint32_t height_)
  {
    width = width_;
    height = height_;
    points.reset( new Point[width*height] );
  }

  CvMat cvMat()
  {
    return ::cvMat(height, width, CV_32FC4, points.get());
  }
};

} //namespace new_point_cloud

#endif
